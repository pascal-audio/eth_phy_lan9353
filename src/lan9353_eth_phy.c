// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_eth.h"

#include "eth_phy_regs_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <driver/i2c.h>

#include "lan9353_eth_phy.h"

static const char *TAG = "lan9353";
#define PHY_CHECK(a, str, result, ...)                                            \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            return result;                                                        \
        }                                                                         \
    } while (0)

#define LAN9353_I2C_PORT CONFIG_LAN9353_I2C_PORT
#define LAN9353_I2C_ADDRESS CONFIG_LAN9353_I2C_ADDRESS

static const bool ACK_CHECK_EN = 0x1; /*!< I2C master will check ack from slave*/
// static const bool ACK_CHECK_DIS = 0x0; /*!< I2C master will not check ack from slave */

#define REG_BYTE_ORDER_TEST 0x0064
#define REG_PORT1_BASIC_STATUS 0x00C4
#define REG_PORT2_BASIC_STATUS 0x01C4
#define REG_PMI_DATA 0xA4
#define REG_PMI_ACCESS 0xA8
#define REG_BYTE_TEST 0x0064
#define LAN9353_BYTE_TEST_DEFAULT 0x87654321
#define LAN9353_HW_CFG 0x0074

#define LAN9353_HW_CFG_DEVICE_READY 0x08000000

#define LAN9353_SWE_INGRSS_PORT_TYP 0x1847
#define LAN9353_BM_EGRSS_PORT_TYPE 0x1C0C
#define LAN9353_SWE_PORT_MIRROR 0x1846

#define LAN9353_SWE_PORT_STATE_PORT2_FORWARDING 0x00000000
#define LAN9353_SWE_PORT_STATE_PORT1_FORWARDING 0x00000000
#define LAN9353_SWE_PORT_STATE_PORT0_FORWARDING 0x00000000

#define LAN9353_SWE_PORT_STATE 0x1843

#define LAN9353_MAC_RX_CFG_REJECT_MAC_TYPES 0x00000002
#define LAN9353_MAC_RX_CFG_RX_EN 0x00000001

#define LAN9353_MAC_TX_CFG_IFG_CONFIG_DEFAULT 0x00000054
#define LAN9353_MAC_TX_CFG_TX_PAD_EN 0x00000002
#define LAN9353_MAC_TX_CFG_TX_EN 0x00000001

#define LAN9353_MAC_RX_CFG(port) (0x0401 + ((port)*0x0400))
#define LAN9353_MAC_TX_CFG(port) (0x0440 + ((port)*0x0400))

#define LAN9353_SWITCH_CSR_CMD 0x01B0
#define LAN9353_SWITCH_CSR_DATA 0x01AC

#define LAN9353_SWITCH_CSR_CMD_BUSY 0x80000000
#define LAN9353_SWITCH_CSR_CMD_WRITE 0x00000000
#define LAN9353_SWITCH_CSR_CMD_READ 0x40000000
#define LAN9353_SWITCH_CSR_CMD_AUTO_INC 0x20000000
#define LAN9353_SWITCH_CSR_CMD_AUTO_DEC 0x10000000
#define LAN9353_SWITCH_CSR_CMD_BE 0x000F0000

#define LAN9353_SWITCH_CSR_CMD_BE_0 0x00010000
#define LAN9353_SWITCH_CSR_CMD_BE_1 0x00020000
#define LAN9353_SWITCH_CSR_CMD_BE_2 0x00040000
#define LAN9353_SWITCH_CSR_CMD_BE_3 0x00080000

#define LAN9353_SWITCH_CSR_CMD_ADDR 0x0000FFFF

/***************Vendor Specific Register***************/

/**
 * @brief PC2R(PHY Special Control/Status Register)
 *
 */
typedef union
{
    struct
    {
        uint32_t sqeoff : 1;                /* Signal Quality Error */
        uint32_t reserved_1 : 1;            /* Reserved */
        uint32_t current_speed : 3;         /* Current Speed/Duplex Indication */
        uint32_t clock_strength : 1;        /* RMII Clock Strengh */
        uint32_t clock_direction : 1;       /* RMII Clock Direction */
        uint32_t switch_collosion_test : 1; /* Switch Collision Test */
        uint32_t mode_10 : 2;               /* Mode[1:0] */
        uint32_t turbo : 1;                 /* Turbo Mode Enable */
        uint32_t reserved_2 : 3;            /* Reserved */
        uint32_t loopback : 1;              /* Switch Loopback */
        uint32_t mode_2 : 1;                /* Mode[2] */
    };
    uint32_t val;
} pscsr_reg_t;
#define LAN9353_PSCSR (0x1F)

typedef union
{
    struct
    {
        uint32_t revision : 16; /* Chip Revision */
        uint32_t id : 16;       /* Chip Id */
    };
    uint32_t val;
} chipid_reg_t;
#define LAN9353_CHIPID_REG (0x050)

typedef union
{
    struct
    {
        uint32_t mii_busy : 1;           /* MII Busy */
        uint32_t mii_write : 1;          /* MII Write */
        uint32_t reserved_1 : 4;         /* Reserved */
        uint32_t mii_register_index : 5; /* MII Register Index */
        uint32_t phy_address : 5;        /* PHY Address */
        uint32_t reserved_2 : 16;        /* Reserved */
    };
    uint32_t val;
} pmi_access_reg_t;

typedef struct
{
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    uint32_t addr;
    uint32_t reset_timeout_ms;
    uint32_t autonego_timeout_ms;
    eth_link_t link_status;
    int reset_gpio_num;
    bool link[2];
} phy_lan9353_t;

static bool _lan9353_check_link(esp_eth_phy_t *phy, uint8_t port, bool *has_link)
{
    pscsr_reg_t reg;

    // Check port number
    if (port >= 1 && port <= 2)
    {
        // Read PHY special control/status register
        PHY_CHECK(lan9353_read_phy_reg(phy, port, LAN9353_PSCSR, &reg.val) == ESP_OK, "Error reading PSCSR", false);

        // Check current operation mode
        switch (reg.current_speed)
        {
        case 1: // 10Base-T half-duplex
        case 2: // 100Base-TX half-duplex
        case 5: // 10Base-T full-duplex
        case 6: // 100Base-TX full-duplex
            *has_link = true;
            return true;
        default:
            *has_link = false;
            return true;
        }
    }

    ESP_LOGE(TAG, "Invalid Port");
    return false;
}

static esp_err_t lan9353_update_link_duplex_speed(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_update_link_duplex_speed");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;
    eth_speed_t speed = ETH_SPEED_100M;
    eth_duplex_t duplex = ETH_DUPLEX_FULL;

    eth_link_t link = ETH_LINK_DOWN;

    // Loop through the ports
    for (uint8_t port = 1; port <= 2; port++)
    {
        // Retrieve current link state
        if (!_lan9353_check_link(phy, port, &lan9353->link[port - 1]))
        {
            return ESP_FAIL;
        }

        if (lan9353->link[port - 1])
        {
            link = ETH_LINK_UP;
        }
    }

    /* check if link status changed */
    if (lan9353->link_status != link)
    {
        /* when link up, read negotiation result */
        if (link == ETH_LINK_UP)
        {
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed) == ESP_OK, "change speed failed", ESP_FAIL);
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex) == ESP_OK, "change duplex failed", ESP_FAIL);
        }

        PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link) == ESP_OK, "change link failed", ESP_FAIL);
        lan9353->link_status = link;
    }

    return ESP_OK;
}

static esp_err_t lan9353_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    PHY_CHECK(eth, "can't set mediator to null", ESP_ERR_INVALID_ARG);
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->eth = eth;
    return ESP_OK;
}

static esp_err_t lan9353_get_link(esp_eth_phy_t *phy)
{
    /* Update information about link, speed, duplex */
    PHY_CHECK(lan9353_update_link_duplex_speed(phy) == ESP_OK, "update link duplex speed failed", ESP_FAIL);

    return ESP_OK;
}

static esp_err_t lan9353_reset(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_reset");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->link_status = ETH_LINK_DOWN;

    bmcr_reg_t bmcr = {.reset = 1};
    PHY_CHECK(lan9353_write_phy_reg(phy, 0, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", ESP_FAIL);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* wait for reset complete */
    bool reset_complete = false;
    uint32_t to;
    for (to = 0; to < lan9353->reset_timeout_ms / 10; to++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        uint32_t value;
        PHY_CHECK(lan9353_read_device_reg(phy, REG_BYTE_TEST, &value, 1) == ESP_OK, "read BOT failed", ESP_FAIL);

        if (value != LAN9353_BYTE_TEST_DEFAULT)
        {
            continue;
        }

        // Read HW_CFG register
        PHY_CHECK(lan9353_read_device_reg(phy, LAN9353_HW_CFG, &value, 1) == ESP_OK, "read HWCFG failed", ESP_FAIL);

        if ((value & LAN9353_HW_CFG_DEVICE_READY) != 0)
        {
            reset_complete = true;
            break;
        }
    }

    PHY_CHECK(reset_complete, "reset timeout", ESP_FAIL);
    return ESP_OK;
}

static esp_err_t lan9353_reset_hw(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_reset_hw");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    if (lan9353->reset_gpio_num >= 0)
    {
        gpio_pad_select_gpio(lan9353->reset_gpio_num);
        gpio_set_direction(lan9353->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(lan9353->reset_gpio_num, 0);
        ets_delay_us(200); // insert min input assert time
        gpio_set_level(lan9353->reset_gpio_num, 1);
    }

    return ESP_OK;
}

static esp_err_t lan9353_negotiate(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_negotiate");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    /* in case any link status has changed, let's assume we're in link down status */
    lan9353->link_status = ETH_LINK_DOWN;

    /* Disable auto negotiation - and set network speed */
    bmcr_reg_t bmcr = {
        .speed_select = 1,     /* 100Mbps */
        .duplex_mode = 1,      /* Full Duplex */
        .en_auto_nego = 0,     /* Auto Negotiation */
        .restart_auto_nego = 0 /* Restart Auto Negotiation */
    };
    PHY_CHECK(lan9353_write_phy_reg(phy, 0, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", ESP_FAIL);

    return ESP_OK;
}

static esp_err_t lan9353_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    ESP_LOGD(TAG, "lan9353_pwrctl, enable=%d", enable ? 1 : 0);

#ifdef LAN9353_POWER_MANAGEMENT
    // Set Power Down Mode
    bmcr_reg_t bmcr;
    PHY_CHECK(lan9353_read_phy_reg(phy, 0, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", ESP_FAIL);
    if (!enable)
    {
        /* General Power Down Mode */
        bmcr.power_down = 1;
    }
    else
    {
        /* Normal operation Mode */
        bmcr.power_down = 0;
    }
    PHY_CHECK(lan9353_write_phy_reg(phy, 0, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", ESP_FAIL);
    PHY_CHECK(lan9353_read_phy_reg(phy, 0, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", ESP_FAIL);

    if (!enable)
    {
        PHY_CHECK(bmcr.power_down == 1, "power down failed", ESP_FAIL);
    }
    else
    {
        PHY_CHECK(bmcr.power_down == 0, "power up failed", ESP_FAIL);
    }
#endif

    return ESP_OK;
}

static esp_err_t lan9353_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    ESP_LOGD(TAG, "lan9353_set_addr");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->addr = addr;
    return ESP_OK;
}

static esp_err_t lan9353_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    PHY_CHECK(addr, "addr can't be null", ESP_ERR_INVALID_ARG);
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    *addr = lan9353->addr;

    return ESP_OK;
}

static esp_err_t lan9353_del(esp_eth_phy_t *phy)
{
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    free(lan9353);
    return ESP_OK;
}

static esp_err_t lan9353_init(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_init");

    /* Power on Ethernet PHY */
    PHY_CHECK(lan9353_pwrctl(phy, true) == ESP_OK, "power control failed", ESP_FAIL);

    /* Reset Ethernet PHY */
    PHY_CHECK(lan9353_reset(phy) == ESP_OK, "reset failed", ESP_FAIL);

    /* Check PHY ID */
    chipid_reg_t chip_id;
    PHY_CHECK(lan9353_read_device_reg(phy, LAN9353_CHIPID_REG, &(chip_id.val), 1) == ESP_OK, "read Chip ID failed", ESP_FAIL);

    ESP_LOGD(TAG, "chip_id=%d, revision=%d", chip_id.id, chip_id.revision);
    PHY_CHECK(chip_id.id == 0x9353, "wrong chip ID", ESP_FAIL);

    // Disable special VLAN tagging mode
    uint32_t value = 0;
    PHY_CHECK(lan9353_write_switch_reg(phy, LAN9353_SWE_INGRSS_PORT_TYP, value) == ESP_OK, "write failed", ESP_FAIL);

    // Revert to default configuration
    value = 0;
    PHY_CHECK(lan9353_write_switch_reg(phy, LAN9353_BM_EGRSS_PORT_TYPE, value) == ESP_OK, "write failed", ESP_FAIL);

    // Disable port mirroring
    value = 0;
    PHY_CHECK(lan9353_write_switch_reg(phy, LAN9353_SWE_PORT_MIRROR, value) == ESP_OK, "write failed", ESP_FAIL);

    // Configure port state
    value = LAN9353_SWE_PORT_STATE_PORT2_FORWARDING | LAN9353_SWE_PORT_STATE_PORT1_FORWARDING | LAN9353_SWE_PORT_STATE_PORT0_FORWARDING;
    PHY_CHECK(lan9353_write_switch_reg(phy, LAN9353_SWE_PORT_STATE, value) == ESP_OK, "write failed", ESP_FAIL);

    return ESP_OK;
}

static esp_err_t lan9353_deinit(esp_eth_phy_t *phy)
{
    ESP_LOGD(TAG, "lan9353_deinit");

    /* Power off Ethernet PHY */
    PHY_CHECK(lan9353_pwrctl(phy, false) == ESP_OK, "power control failed", ESP_FAIL);

    return ESP_OK;
}

esp_eth_phy_t *esp_eth_phy_new_lan9353(const eth_phy_config_t *config)
{
    ESP_LOGD(TAG, "lan9353_new");

    PHY_CHECK(config, "can't set phy config to null", NULL);
    phy_lan9353_t *lan9353 = calloc(1, sizeof(phy_lan9353_t));
    PHY_CHECK(lan9353, "calloc lan9353 failed", NULL);
    lan9353->addr = config->phy_addr;
    lan9353->reset_gpio_num = config->reset_gpio_num;
    lan9353->reset_timeout_ms = config->reset_timeout_ms;
    lan9353->link_status = ETH_LINK_DOWN;
    lan9353->autonego_timeout_ms = config->autonego_timeout_ms;
    lan9353->parent.reset = lan9353_reset;
    lan9353->parent.reset_hw = lan9353_reset_hw;
    lan9353->parent.init = lan9353_init;
    lan9353->parent.deinit = lan9353_deinit;
    lan9353->parent.set_mediator = lan9353_set_mediator;
    lan9353->parent.negotiate = lan9353_negotiate;
    lan9353->parent.get_link = lan9353_get_link;
    lan9353->parent.pwrctl = lan9353_pwrctl;
    lan9353->parent.get_addr = lan9353_get_addr;
    lan9353->parent.set_addr = lan9353_set_addr;
    lan9353->parent.del = lan9353_del;

    return &(lan9353->parent);
}

static uint32_t _byteswap32(uint32_t x)
{
    uint32_t y = (x >> 24) & 0xff;
    y |= (x >> 8) & 0xff00;
    y |= (x << 8) & 0xff0000;
    y |= (x << 24) & 0xff000000;

    return y;
}

esp_err_t lan9353_read_device_reg(esp_eth_phy_t *phy, uint32_t reg, uint32_t *val, size_t count)
{
    i2c_set_timeout(LAN9353_I2C_PORT, 0xFFFFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LAN9353_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (reg >> 2) & 0xFF, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LAN9353_I2C_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_read(cmd, (uint8_t *)val, 4 * count, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));

    i2c_cmd_link_delete(cmd);

    for (uint32_t i = 0; i < count; i++)
    {
        val[i] = _byteswap32(val[i]);
    }

    return result;
}

esp_err_t lan9353_write_device_reg(esp_eth_phy_t *phy, uint32_t reg, uint32_t *val, size_t count)
{
    i2c_set_timeout(I2C_NUM_0, 0xFFFFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LAN9353_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (reg >> 2) & 0xFF, ACK_CHECK_EN));
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t tmp = _byteswap32(val[i]);
        ESP_ERROR_CHECK(i2c_master_write(cmd, (uint8_t *)&tmp, 4, ACK_CHECK_EN));
    }
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));

    i2c_cmd_link_delete(cmd);

    return result;
}

static esp_err_t _lan9353_get_vphy_addr(uint32_t addr, uint32_t index)
{
    assert(addr == 0);
    uint16_t reg_base;

    switch (index)
    {
    case 0:
        reg_base = 0x0C0;
        break;
    case 1:
        reg_base = 0x0C4;
        break;
    case 2:
        reg_base = 0x0C8;
        break;
    case 3:
        reg_base = 0x0CC;
        break;
    case 4:
        reg_base = 0x0D0;
        break;
    case 5:
        reg_base = 0x0D4;
        break;
    case 6:
        reg_base = 0x0D8;
        break;
    case 31:
        reg_base = 0x0DC;
        break;
    default:
        assert(false);
    }

    return reg_base + ((addr == 0) ? 0x100 : 0x000);
}

static esp_err_t _lan9353_write_i2c_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t val)
{
    pmi_access_reg_t pmi_access = {0};
    pmi_access.phy_address = addr;
    pmi_access.mii_register_index = index;
    pmi_access.mii_write = 1;

    PHY_CHECK(lan9353_write_device_reg(phy, REG_PMI_DATA, &val, 1) == ESP_OK, "write failed", ESP_FAIL);
    // usleep(1000);
    PHY_CHECK(lan9353_write_device_reg(phy, REG_PMI_ACCESS, &pmi_access.val, 1) == ESP_OK, "write failed", ESP_FAIL);
    // usleep(1000);

    return ESP_OK;
}

static esp_err_t _lan9353_write_i2c_vphy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t val)
{
    pmi_access_reg_t pmi_access = {0};
    pmi_access.phy_address = addr;
    pmi_access.mii_register_index = index;
    pmi_access.mii_write = 1;

    PHY_CHECK(lan9353_write_device_reg(phy, REG_PMI_DATA, &val, 1) == ESP_OK, "write failed", ESP_FAIL);
    // usleep(1000);
    PHY_CHECK(lan9353_write_device_reg(phy, REG_PMI_ACCESS, &pmi_access.val, 1) == ESP_OK, "write failed", ESP_FAIL);
    // usleep(1000);

    return ESP_OK;
}

esp_err_t lan9353_write_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t val)
{
#ifdef CONFIG_LAN9353_MGMT_I2C
    switch (addr)
    {
    case 0:
        return _lan9353_write_i2c_vphy_reg(phy, addr, index, val);
    case 1:
    case 2:
        return _lan9353_write_i2c_phy_reg(phy, addr, index, val);
    default:
        return ESP_ERR_INVALID_ARG;
    }
#else
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;

    return eth->phy_write_reg(eth, lan9353->addr, index, val);
#endif
}

static esp_err_t _lan9353_read_i2c_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t *val)
{
    pmi_access_reg_t pmi_access = {0};
    pmi_access.phy_address = addr;
    pmi_access.mii_register_index = index;

    PHY_CHECK(lan9353_write_device_reg(phy, REG_PMI_ACCESS, &pmi_access.val, 1) == ESP_OK, "write failed", ESP_FAIL);
    ets_delay_us(100);
    PHY_CHECK(lan9353_read_device_reg(phy, REG_PMI_DATA, val, 1) == ESP_OK, "read failed", ESP_FAIL);
    ets_delay_us(100);

    return ESP_OK;
}

static esp_err_t _lan9353_read_i2c_vphy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t *val)
{
    uint16_t reg = _lan9353_get_vphy_addr(addr, index);

    return lan9353_read_device_reg(phy, reg, val, 1);
}

esp_err_t lan9353_read_phy_reg(esp_eth_phy_t *phy, uint32_t addr, uint32_t index, uint32_t *val)
{
#ifdef CONFIG_LAN9353_MGMT_I2C
    switch (addr)
    {
    case 0:
        return _lan9353_read_i2c_vphy_reg(phy, addr, index, val);
    case 1:
    case 2:
        return _lan9353_read_i2c_phy_reg(phy, addr, index, val);
    default:
        return ESP_ERR_INVALID_ARG;
    }
#else
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;

    return lan9353_read_phy_reg(phy, 0, index, val);
#endif
}

/**
 * @brief Write switch fabric CSR register
 **/

esp_err_t lan9353_write_switch_reg(esp_eth_phy_t *phy, uint16_t address, uint32_t data)
{
    uint32_t value;

    // To perform a write to an individual switch fabric register, the desired
    // data must first be written into the SWITCH_CSR_DATA register
    PHY_CHECK(lan9353_write_device_reg(phy, LAN9353_SWITCH_CSR_DATA, &data, 1) == ESP_OK, "write failed", ESP_FAIL);

    // Set up a write operation
    value = LAN9353_SWITCH_CSR_CMD_BUSY | LAN9353_SWITCH_CSR_CMD_WRITE |
            LAN9353_SWITCH_CSR_CMD_BE;

    // Set register address
    value |= address & LAN9353_SWITCH_CSR_CMD_ADDR;

    // The write cycle is initiated by performing a single write to the
    // SWITCH_CSR_CMD register
    PHY_CHECK(lan9353_write_device_reg(phy, LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "write failed", ESP_FAIL);

    // The completion of the write cycle is indicated by the clearing of the
    // CSR_BUSY bit
    do
    {
        // Read SWITCH_CSR_CMD register
        PHY_CHECK(lan9353_read_device_reg(phy, LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "read failed", ESP_FAIL);

        // Poll CSR_BUSY bit
    } while ((value & LAN9353_SWITCH_CSR_CMD_BUSY) != 0);

    return ESP_OK;
}

esp_err_t lan9353_read_switch_reg(esp_eth_phy_t *phy, uint16_t address, uint32_t *val)
{
    // Set up a read operation
    uint32_t value = LAN9353_SWITCH_CSR_CMD_BUSY | LAN9353_SWITCH_CSR_CMD_READ |
                     LAN9353_SWITCH_CSR_CMD_BE;

    // Set register address
    value |= address & LAN9353_SWITCH_CSR_CMD_ADDR;

    // To perform a read of an individual switch fabric register, the read cycle
    // must be initiated by performing a single write to the SWITCH_CSR_CMD
    // register
    PHY_CHECK(lan9353_write_device_reg(phy, LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "write failed", ESP_FAIL);

    // Valid data is available for reading when the CSR_BUSY bit is cleared
    do
    {
        // Read SWITCH_CSR_CMD register
        PHY_CHECK(lan9353_read_device_reg(phy, LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "read failed", ESP_FAIL);

        // Poll CSR_BUSY bit
    } while ((value & LAN9353_SWITCH_CSR_CMD_BUSY) != 0);

    // Read data from the SWITCH_CSR_DATA register
    return lan9353_read_device_reg(phy, LAN9353_SWITCH_CSR_DATA, val, 1);
}

bool lan9353_get_link_status(esp_eth_phy_t *phy, uint8_t port)
{
    assert(port < 2);

    ESP_LOGD(TAG, "lan9353_get_link_status");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);

    return lan9353->link[port];
}