import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.zephyr import (
    zephyr_add_overlay,
    zephyr_add_prj_conf,
)

from esphome.const import CONF_ID

ld2410s_ns = cg.esphome_ns.namespace("ld2410s")
LD2410SComponent = ld2410s_ns.class_("LD2410SComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LD2410SComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    
    zephyr_add_prj_conf("CONFIG_PM_DEVICE", True)

    zephyr_add_overlay(
        """
            / {
                aliases {
                    uart-mmwave = &uart0;
                    ldo-enable = &ldo_enable;
                };

                sensor_interrupt {
                    compatible = "gpio-keys";
                    #address-cells = <1>;
                    #size-cells = <0>;
                    status = "okay";

                    interrupt_pin: interrupt_pin {
                        label = "Sensor Interrupt";
                        gpios = <&xiao_d 3 GPIO_ACTIVE_HIGH>;
                    };
                };

                gpio_outputs {
                    compatible = "gpio-leds";
                    ldo_enable: ldo_enable {
                        label = "LDO Enable";
                        gpios = <&xiao_d 2 GPIO_ACTIVE_HIGH>;
                    };
                };
            };
        """
    )
