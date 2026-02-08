#pragma once

#include "esphome/core/component.h"
#include "ld2410s.h"

namespace esphome {
namespace ld2410s {

static const char *TAG = "ld2410s";

class LD2410SComponent : public Component {
 public:
  // Command sequence system for scalability
  struct CommandStep {
    void (*send_func)();     // Function to send the command
    uint8_t expected_ack;    // Expected ACK command code
    const char *name;        // Command name for logging
  };

  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:

  enum State {
    IDLE,
    EXEC_SEQUENCE_SEND,      // Send current command in sequence
    EXEC_SEQUENCE_WAIT_ACK,  // Wait for ACK of current command
  };

  State state_{IDLE};
  const CommandStep *current_sequence_{nullptr};  // Current command sequence being executed
  uint8_t sequence_length_{0};    // Total commands in sequence
  uint8_t sequence_index_{0};     // Current position in sequence
  uint32_t ack_timeout_start_{0};
  uint8_t retry_count_{0};

  // Execute a command sequence (open, commands, close)
  void execute_sequence_(const CommandStep *sequence, uint8_t length);

  // Handle ACK waiting with timeout and retry logic
  void handle_sequence_ack_();
};

}  // namespace ld2410s
}  // namespace esphome
