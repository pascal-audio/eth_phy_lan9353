#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "esp_eth_com.h"
#include "sdkconfig.h"

    /**
     * @brief Create a PHY instance of lan9353
     *
     * @param[in] config: configuration of PHY
     *
     * @return
     *      - instance: create PHY instance successfully
     *      - NULL: create PHY instance failed because some error occurred
     */
    esp_eth_phy_t *esp_eth_phy_new_lan9353(const eth_phy_config_t *config);

    esp_err_t lan9353_read_device_reg(esp_eth_phy_t *phy, uint32_t reg, uint32_t *val, size_t count);
    esp_err_t lan9353_write_device_reg(esp_eth_phy_t *phy, uint32_t reg, uint32_t *val, size_t count);

    esp_err_t lan9353_read_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t *val);
    esp_err_t lan9353_write_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t val);

    esp_err_t lan9353_read_switch_reg(esp_eth_phy_t *phy, uint16_t address, uint32_t *val);
    esp_err_t lan9353_write_switch_reg(esp_eth_phy_t *phy, uint16_t address, uint32_t data);

    esp_err_t lan9353_dump(esp_eth_phy_t *phy);

    static bool lan9353_get_link_status(esp_eth_phy_t *phy, uint8_t port);

#ifdef __cplusplus
}
#endif