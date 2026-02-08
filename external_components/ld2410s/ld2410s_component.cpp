#include "ld2410s_component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "ld2410s.h"

namespace esphome {
namespace ld2410s {

  static LD2410SComponent::CommandStep step3_sequence[] = {
    {ld2410s_send_open_cmd, 0xFF, "OPEN"},
    {ld2410s_send_general_config_cmd, 0x70, "CONFIG"},
    {ld2410s_send_detection_distance_cmd, 0x70, "DETECTION_DISTANCE"},
    {ld2410s_send_disappearance_delay_cmd, 0x70, "DISAPPEARANCE_DELAY"},
    {ld2410s_send_data_format_cmd, 0x7A, "DATA_FORMAT"},
    {ld2410s_send_read_config_cmd, 0x71, "READ_CONFIG"},
    {ld2410s_send_get_thresholds_cmd, 0x73, "GET_THRESHOLDS"},
    {ld2410s_send_set_thresholds_cmd, 0x72, "SET_THRESHOLDS"},
    {ld2410s_send_get_thresholds_cmd, 0x73, "GET_THRESHOLDS_VERIFY"},
    {ld2410s_send_close_cmd, 0xFE, "CLOSE"},
  };

  void LD2410SComponent::setup() {
    ESP_LOGI(TAG, "Setup...");
    ld2410s_init();

    execute_sequence_(step3_sequence, sizeof(step3_sequence) / sizeof(step3_sequence[0]));

    ESP_LOGI(TAG, "Setup done, starting Step 3 sequence");
  }

  // Start executing a command sequence
  void LD2410SComponent::execute_sequence_(const CommandStep *sequence, uint8_t length) {
    this->current_sequence_ = sequence;
    this->sequence_length_ = length;
    this->sequence_index_ = 0;
    this->retry_count_ = 0;

    // Resume UART when leaving IDLE state
    if (this->state_ == IDLE) {
      ld2410s_uart_resume();
    }

    this->state_ = EXEC_SEQUENCE_SEND;

    ESP_LOGI(TAG, "Starting command sequence (%u commands)", length);
  }

  // Handle ACK waiting with timeout and retry
  void LD2410SComponent::handle_sequence_ack_() {
    if (this->current_sequence_ == nullptr) {
      ESP_LOGE(TAG, "No active sequence!");
      this->state_ = IDLE;
      ld2410s_uart_suspend();
      return;
    }

    const CommandStep &step = this->current_sequence_[this->sequence_index_];
    struct ack_frame ack_frame;

    // Try to get ACK from queue
    if (ld2410s_get_ack(&ack_frame)) {
      uint32_t elapsed = millis() - this->ack_timeout_start_;

      if (ack_frame.cmd == step.expected_ack && ack_frame.success) {
        ESP_LOGI(TAG, "%s ACK received after %u ms - success", step.name, elapsed);

        if (step.expected_ack == 0x71) {
          if (ack_frame.data_size >= 17) {
            ESP_LOGI(TAG,
                     "READ_CONFIG values: detection_distance=%u disappearance_delay=%u status_freq=%u distance_freq=%u response_speed=0x%02X",
                     ack_frame.data[0], ack_frame.data[4], ack_frame.data[8], ack_frame.data[12], ack_frame.data[16]);
          } else {
            ESP_LOGW(TAG, "READ_CONFIG payload too short: %u bytes", ack_frame.data_size);
          }
        } else if (step.expected_ack == 0x73) {
          if (ack_frame.data_size >= 64) {
            for (int i = 0; i < 16; i++) {
              uint32_t threshold =
                  ((uint32_t) ack_frame.data[i * 4]) |
                  ((uint32_t) ack_frame.data[i * 4 + 1] << 8) |
                  ((uint32_t) ack_frame.data[i * 4 + 2] << 16) |
                  ((uint32_t) ack_frame.data[i * 4 + 3] << 24);
              ESP_LOGI(TAG, "%s gate[%d]=%u", step.name, i, threshold);
            }
          } else {
            ESP_LOGW(TAG, "%s payload too short: %u bytes", step.name, ack_frame.data_size);
          }
        } else if (step.expected_ack == 0x72) {
          ESP_LOGI(TAG, "SET_THRESHOLDS ACK success");
        }

        // Move to next command in sequence
        this->sequence_index_++;
        this->retry_count_ = 0;

        if (this->sequence_index_ >= this->sequence_length_) {
          // Sequence complete
          ESP_LOGI(TAG, "Command sequence complete!");
          this->current_sequence_ = nullptr;
          this->state_ = IDLE;
          ld2410s_uart_suspend();
        } else {
          // Send next command
          this->state_ = EXEC_SEQUENCE_SEND;
        }
        return;
      } else if (ack_frame.cmd == step.expected_ack && !ack_frame.success) {
        ESP_LOGW(TAG, "%s ACK received but indicates failure", step.name);
        if (this->retry_count_ < 3) {
          this->retry_count_++;
          ESP_LOGI(TAG, "Retrying %s (attempt %u/3)...", step.name, this->retry_count_);
          this->state_ = EXEC_SEQUENCE_SEND;
        } else {
          ESP_LOGE(TAG, "%s failed after 3 retries, aborting sequence", step.name);
          this->current_sequence_ = nullptr;
          this->state_ = IDLE;
          ld2410s_uart_suspend();
        }
        return;
      } else {
        ESP_LOGW(TAG, "ACK mismatch: expected 0x%02X, got 0x%02X - ignoring",
                 step.expected_ack, ack_frame.cmd);
      }
    }

    // Check for timeout
    uint32_t elapsed = millis() - this->ack_timeout_start_;
    if (elapsed > 2000) {
      if (this->retry_count_ < 3) {
        this->retry_count_++;
        ESP_LOGW(TAG, "%s ACK timeout after %u ms, retry %u/3",
                 step.name, elapsed, this->retry_count_);
        this->state_ = EXEC_SEQUENCE_SEND;
      } else {
        ESP_LOGE(TAG, "%s ACK timeout after 3 retries, aborting sequence", step.name);
        this->current_sequence_ = nullptr;
        this->state_ = IDLE;
        ld2410s_uart_suspend();
      }
    }
  }

  void LD2410SComponent::loop() {
    /* Parses received UART data */
    ld2410s_process_uart_data();

    switch (this->state_) {
      case IDLE:
        // Nothing to do
        break;

      case EXEC_SEQUENCE_SEND: {
        if (this->current_sequence_ == nullptr) {
          ESP_LOGE(TAG, "No active sequence!");
          this->state_ = IDLE;
          ld2410s_uart_suspend();
          break;
        }

        const CommandStep &step = this->current_sequence_[this->sequence_index_];
        ESP_LOGI(TAG, "[%u/%u] Sending %s command (0x%02X)...",
                 this->sequence_index_ + 1, this->sequence_length_,
                 step.name, step.expected_ack);

        // Call the command's send function
        step.send_func();

        this->ack_timeout_start_ = millis();
        this->state_ = EXEC_SEQUENCE_WAIT_ACK;
        break;
      }

      case EXEC_SEQUENCE_WAIT_ACK:
        handle_sequence_ack_();
        break;
    }
  }

}  // namespace ld2410s
}  // namespace esphome
