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
#define PHY_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define LAN9353_I2C_PORT 0
#define LAN9353_I2C_ADDRESS 0x0A

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
        uint32_t reserved_1 : 2; /* Reserved */
        uint32_t speed_ind : 3;  /* Speed Indication */
        uint32_t reserved_2 : 7; /* Reserved */
        uint32_t auto_done : 1;  /* Autodone */
        uint32_t reserved_3 : 3; /* Reserved */
    };
    uint32_t val;
} pscsr_reg_t;
#define ETH_PHY_SCSR_REG_ADDR (0x1F)

typedef struct
{
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    uint32_t addr;
    uint32_t reset_timeout_ms;
    uint32_t autonego_timeout_ms;
    eth_link_t link_status;
    int reset_gpio_num;
} phy_lan9353_t;

static esp_err_t lan9353_update_link_duplex_speed(phy_lan9353_t *lan9353)
{
    ESP_LOGD(TAG, "lan9353_update_link_duplex_speed");

    esp_eth_mediator_t *eth = lan9353->eth;
    eth_speed_t speed = ETH_SPEED_100M;
    eth_duplex_t duplex = ETH_DUPLEX_FULL;
    bmsr_reg_t bmsr;
    pscsr_reg_t pscsr;

    // PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK,
    //           "read BMSR failed", err);
    eth_link_t link = ETH_LINK_UP;

    // ESP_LOGI(TAG, "0x01h - link=%d, BMSR: %04x", bmsr.link_status ? 1 : 0, bmsr.val);

    /* check if link status changed */
    if (lan9353->link_status != link)
    {
        /* when link up, read negotiation result */
        if (link == ETH_LINK_UP)
        {
            // PHY_CHECK(lan9353_read_phy_reg(ETH_PHY_SCSR_REG_ADDR, 0, &pscsr.val) == ESP_OK, "read PSCSR failed", err);

            // PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_SCSR_REG_ADDR, &(pscsr.val)) == ESP_OK,
            //           "read PSCSR failed", err);

            // ESP_LOGI(TAG, "0x1Fh (PSCSR)=%04x", pscsr.val);
            // ESP_LOGI(TAG, "speed=%02x", pscsr.speed_ind);
            // switch (pscsr.speed_ind)
            // {
            // case 1: // 10Base-T half-duplex
            //     ESP_LOGI(TAG, "10M HalfDuplex");

            //     speed = ETH_SPEED_10M;
            //     duplex = ETH_DUPLEX_HALF;
            //     break;
            // case 2: // 100Base-TX half-duplex
            //     ESP_LOGI(TAG, "100M HalfDuplex");
            //     speed = ETH_SPEED_100M;
            //     duplex = ETH_DUPLEX_HALF;
            //     break;
            // case 5: // 10Base-T full-duplex
            //     ESP_LOGI(TAG, "10M FullDuplex");
            //     speed = ETH_SPEED_10M;
            //     duplex = ETH_DUPLEX_FULL;
            //     break;
            // case 6: // 100Base-TX full-duplex
            //     ESP_LOGI(TAG, "100M FullDuplex");
            //     speed = ETH_SPEED_100M;
            //     duplex = ETH_DUPLEX_FULL;
            //     break;
            // default:
            //     ESP_LOGI(TAG, "UNKNOWN LINK SPEED");
            //     break;
            // }
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed) == ESP_OK,
                      "change speed failed", err);
            PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex) == ESP_OK,
                      "change duplex failed", err);
        }
        PHY_CHECK(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link) == ESP_OK,
                  "change link failed", err);
        lan9353->link_status = link;
    }

    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t lan9353_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    PHY_CHECK(eth, "can't set mediator to null", err);
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->eth = eth;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t lan9353_get_link(esp_eth_phy_t *phy)
{
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    /* Update information about link, speed, duplex */
    PHY_CHECK(lan9353_update_link_duplex_speed(lan9353) == ESP_OK, "update link duplex speed failed", err);

    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t lan9353_reset(esp_eth_phy_t *phy)
{
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->link_status = ETH_LINK_DOWN;
    esp_eth_mediator_t *eth = lan9353->eth;
    bmcr_reg_t bmcr = {.reset = 1};
    PHY_CHECK(eth->phy_reg_write(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK,
              "write BMCR failed", err);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* wait for reset complete */
    uint32_t to = 0;
    for (to = 0; to < lan9353->reset_timeout_ms / 10; to++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "read BMCR");
        PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK,
                  "read BMCR failed", err);
        ESP_LOGI(TAG, "BMCR=%08X", bmcr.val);
        if (!bmcr.reset)
        {
            break;
        }
    }
    PHY_CHECK(to < lan9353->reset_timeout_ms / 10, "reset timeout", err);
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t lan9353_reset_hw(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "lan9353_reset_hw");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    if (lan9353->reset_gpio_num >= 0)
    {
        gpio_pad_select_gpio(lan9353->reset_gpio_num);
        gpio_set_direction(lan9353->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(lan9353->reset_gpio_num, 0);
        ets_delay_us(1000); // insert min input assert time
        gpio_set_level(lan9353->reset_gpio_num, 1);
    }

    ESP_LOGI(TAG, "lan9353_reset_hw.done");

    return ESP_OK;
}

static esp_err_t lan9353_negotiate(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "lan9353_negotiate");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;
    /* in case any link status has changed, let's assume we're in link down status */
    lan9353->link_status = ETH_LINK_DOWN;
    /* Restart auto negotiation */
    bmcr_reg_t bmcr = {
        .speed_select = 1,     /* 100Mbps */
        .duplex_mode = 1,      /* Full Duplex */
        .en_auto_nego = 0,     /* Auto Negotiation */
        .restart_auto_nego = 0 /* Restart Auto Negotiation */
    };
    PHY_CHECK(eth->phy_reg_write(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK, "write BMCR failed", err);

#ifdef AUTONEG
    /* Wait for auto negotiation complete */
    bmsr_reg_t bmsr;
    int32_t to = 0;
    for (to = 0; to < lan9353->autonego_timeout_ms / 100; to++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)) == ESP_OK,
                  "read BMSR failed", err);
        if (bmsr.auto_nego_complete)
        {
            break;
        }
    }
    /* Auto negotiation failed, maybe no network cable plugged in, so output a warning */
    if (to >= lan9353->autonego_timeout_ms / 100)
    {
        ESP_LOGW(TAG, "auto negotiation timeout");
    }
#endif
    return ESP_OK;
err:
    return ESP_FAIL;
}

static esp_err_t lan9353_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    ESP_LOGI(TAG, "lan9353_pwrctl, enable=%d", enable ? 1 : 0);

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;

    // Set Power Down Mode
    bmcr_reg_t bmcr;
    PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK, "read BMCR failed", err);
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
    PHY_CHECK(eth->phy_reg_write(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val) == ESP_OK,
              "write BMCR failed", err);
    PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)) == ESP_OK,
              "read BMCR failed", err);
    if (!enable)
    {
        PHY_CHECK(bmcr.power_down == 1, "power down failed", err);
    }
    else
    {
        PHY_CHECK(bmcr.power_down == 0, "power up failed", err);
    }
    return ESP_OK;

err:
    return ESP_FAIL;
}

static esp_err_t lan9353_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    ESP_LOGI(TAG, "lan9353_set_addr");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    lan9353->addr = addr;
    return ESP_OK;
}

static esp_err_t lan9353_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    PHY_CHECK(addr, "addr can't be null", err);
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    *addr = lan9353->addr;
    return ESP_OK;
err:
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t lan9353_del(esp_eth_phy_t *phy)
{
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    free(lan9353);
    return ESP_OK;
}

static esp_err_t lan9353_init_2(esp_eth_phy_t *phy)
{
    uint32_t value;
    esp_err_t res;

    // Debug message
    ESP_LOGI(TAG, "Initializing LAN9353...");

    // Chip-level reset/configuration completion can be determined by first
    // polling the BYTE_TEST register

    ESP_LOGI(TAG, "Init:1");

    do
    {
        // Read BYTE_TEST register
        res = lan9353_read_i2c_reg(REG_BYTE_TEST, &value, 1);
        ESP_LOGI(TAG, "BOT=%08x, res=%d (%s)", value, res, esp_err_to_name(res));

        if (res != ESP_OK)
        {
            continue;
        }

        // The returned data is invalid until the serial interface reset is
        // complete
    } while (value != LAN9353_BYTE_TEST_DEFAULT);

    ESP_LOGI(TAG, "Init:2");

    // The completion of the entire chip-level reset must then be determined
    // by polling the READY bit of the HW_CFG register
    do
    {
        // Read HW_CFG register
        res = lan9353_read_i2c_reg(LAN9353_HW_CFG, &value, 1);

        if (res != ESP_OK)
        {
            continue;
        }

        // When set, the READY bit indicates that the reset has completed and
        // the device is ready to be accessed
    } while ((value & LAN9353_HW_CFG_DEVICE_READY) == 0);

    ESP_LOGI(TAG, "Init:3");

    // Disable special VLAN tagging mode
    value = 0;
    PHY_CHECK(lan9353_write_switch_reg(LAN9353_SWE_INGRSS_PORT_TYP, value) == ESP_OK, "write failed", err);

    // Revert to default configuration
    value = 0;
    PHY_CHECK(lan9353_write_switch_reg(LAN9353_BM_EGRSS_PORT_TYPE, value) == ESP_OK, "write failed", err);

    // Disable port mirroring
    value = 0;
    PHY_CHECK(lan9353_write_switch_reg(LAN9353_SWE_PORT_MIRROR, value) == ESP_OK, "write failed", err);

    // Configure port state
    value = LAN9353_SWE_PORT_STATE_PORT2_FORWARDING |
            LAN9353_SWE_PORT_STATE_PORT1_FORWARDING |
            LAN9353_SWE_PORT_STATE_PORT0_FORWARDING;
    PHY_CHECK(lan9353_write_switch_reg(LAN9353_SWE_PORT_STATE, value) == ESP_OK, "write failed", err);

    lan9353_write_phy_reg(1, 16, 0x0001);
    lan9353_write_phy_reg(2, 16, 0x0001);

    uint32_t reg, value1, value2;
    reg = 16;
    res = lan9353_read_phy_reg(1, reg, &value1);
    res = lan9353_read_phy_reg(2, reg, &value2);
    ESP_LOGI(TAG, "PHY_EDPD_CFG_x=                  [ %04x, %04x ]", value1, value2);

    lan9353_write_phy_reg(1, 27, 0xC000);
    lan9353_write_phy_reg(2, 27, 0xC000);

    lan9353_write_phy_reg(1, 0, 0x8000);
    lan9353_write_phy_reg(2, 0, 0x8000);

    // usleep(10000);

    lan9353_write_phy_reg(1, 0, 0x3100);
    lan9353_write_phy_reg(2, 0, 0x3100);

    // usleep(10000);

    res = lan9353_read_phy_reg(1, 27, &value);
    ESP_LOGI(TAG, "PHY_SPECIAL_CONTROL_STAT_IND_1=%08x, res=%d (%s)", value, res, esp_err_to_name(res));

    res = lan9353_read_phy_reg(2, 27, &value);
    ESP_LOGI(TAG, "PHY_SPECIAL_CONTROL_STAT_IND_2=%08x, res=%d (%s)", value, res, esp_err_to_name(res));

    // // Configure port 0 receive parameters
    // value = LAN9353_MAC_RX_CFG_REJECT_MAC_TYPES | LAN9353_MAC_RX_CFG_RX_EN;
    // ESP_RETURN_ON_ERROR(lan9353_write_switch_reg(LAN9353_MAC_RX_CFG(0), value), TAG, "write failed");

    // // Configure port 0 transmit parameters
    // value = LAN9353_MAC_TX_CFG_IFG_CONFIG_DEFAULT |
    //         LAN9353_MAC_TX_CFG_TX_PAD_EN |
    //         LAN9353_MAC_TX_CFG_TX_EN;
    // ESP_RETURN_ON_ERROR(lan9353_write_switch_reg(LAN9353_MAC_TX_CFG(0), value), TAG, "write failed");

    return ESP_OK;

err:
    return ESP_FAIL;
}

static esp_err_t lan9353_init(esp_eth_phy_t *phy)
{
    // ESP_LOGI(TAG, "lan9353_init");

    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;

    /* Power on Ethernet PHY */
    PHY_CHECK(lan9353_pwrctl(phy, true) == ESP_OK, "power control failed", err);
    /* Reset Ethernet PHY */

    PHY_CHECK(lan9353_reset(phy) == ESP_OK, "reset failed", err);
    /* Check PHY ID */
    phyidr1_reg_t id1;
    phyidr2_reg_t id2;
    PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_IDR1_REG_ADDR, &(id1.val)) == ESP_OK,
              "read ID1 failed", err);
    PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, ETH_PHY_IDR2_REG_ADDR, &(id2.val)) == ESP_OK,
              "read ID2 failed", err);

    ESP_LOGI(TAG, "msb=%04x, lsb=%04x, vm=%04x", id1.oui_msb, id2.oui_lsb, id2.vendor_model);
    // PHY_CHECK(id1.oui_msb == 0x22 && id2.oui_lsb == 0x5 && id2.vendor_model == 0x16, "wrong chip ID", err);

    lan9353_init_2(phy);

    return ESP_OK;

err:
    return ESP_FAIL;
}

static esp_err_t lan9353_deinit(esp_eth_phy_t *phy)
{
    ESP_LOGI(TAG, "lan9353_deinit");

    /* Power off Ethernet PHY */
    PHY_CHECK(lan9353_pwrctl(phy, false) == ESP_OK, "power control failed", err);

    return ESP_OK;
err:
    return ESP_FAIL;
}

esp_eth_phy_t *esp_eth_phy_new_lan9353(const eth_phy_config_t *config)
{
    ESP_LOGI(TAG, "create");

    PHY_CHECK(config, "can't set phy config to null", err);
    phy_lan9353_t *lan9353 = calloc(1, sizeof(phy_lan9353_t));
    PHY_CHECK(lan9353, "calloc lan9353 failed", err);
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
err:
    return NULL;
}

void lan9353_dump(esp_eth_phy_t *phy)
{
    phy_lan9353_t *lan9353 = __containerof(phy, phy_lan9353_t, parent);
    esp_eth_mediator_t *eth = lan9353->eth;

    uint32_t regs[9] = {0x00, 0x01, 31};

    for (int i = 0; i < 3; i++)
    {
        int32_t reg = regs[i];
        uint32_t val;
        // PHY_CHECK(eth->phy_reg_read(eth, addr, reg, &val) == ESP_OK,
        //           "read failed", err);
        PHY_CHECK(eth->phy_reg_read(eth, lan9353->addr, reg, &val) == ESP_OK,
                  "read failed", err);
        ESP_LOGI(TAG, "reg 0x%02x=0x%04x", reg, val);
    }

    return;
err:
    return;
}

static uint32_t _byteswap32(uint32_t x)
{
    uint32_t y = (x >> 24) & 0xff;
    y |= (x >> 8) & 0xff00;
    y |= (x << 8) & 0xff0000;
    y |= (x << 24) & 0xff000000;

    return y;
}

esp_err_t lan9353_read_i2c_reg(uint32_t reg, uint32_t *val, uint32_t count)
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

esp_err_t lan9353_write_i2c_reg(uint32_t reg, uint32_t *val, size_t count)
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

esp_err_t lan9353_write_phy_reg(uint32_t addr, uint32_t index, uint32_t val)
{
    uint32_t tmp = 0x0;
    tmp |= 0x02;
    tmp |= addr << 11;
    tmp |= index << 6;

    PHY_CHECK(lan9353_write_i2c_reg(REG_PMI_DATA, &val, 1) == ESP_OK, "write failed", err);
    // usleep(1000);
    PHY_CHECK(lan9353_write_i2c_reg(REG_PMI_ACCESS, &tmp, 1) == ESP_OK, "write failed", err);
    // usleep(1000);

    return ESP_OK;

err:
    return ESP_FAIL;
}

esp_err_t lan9353_read_phy_reg(uint32_t addr, uint32_t index, uint32_t *val)
{
    uint32_t tmp = 0x0;
    tmp |= addr << 11;
    tmp |= index << 6;

    PHY_CHECK(lan9353_write_i2c_reg(REG_PMI_ACCESS, &tmp, 1) == ESP_OK, "write failed", err);
    // usleep(100);
    PHY_CHECK(lan9353_read_i2c_reg(REG_PMI_DATA, val, 1) == ESP_OK, "read failed", err);
    // usleep(100);

    return ESP_OK;

err:
    return ESP_FAIL;
}

/**
 * @brief Write switch fabric CSR register
 **/

esp_err_t lan9353_write_switch_reg(uint16_t address, uint32_t data)
{
    uint32_t value;

    // To perform a write to an individual switch fabric register, the desired
    // data must first be written into the SWITCH_CSR_DATA register
    lan9353_write_i2c_reg(LAN9353_SWITCH_CSR_DATA, &data, 1);

    // Set up a write operation
    value = LAN9353_SWITCH_CSR_CMD_BUSY | LAN9353_SWITCH_CSR_CMD_WRITE |
            LAN9353_SWITCH_CSR_CMD_BE;

    // Set register address
    value |= address & LAN9353_SWITCH_CSR_CMD_ADDR;

    // The write cycle is initiated by performing a single write to the
    // SWITCH_CSR_CMD register
    PHY_CHECK(lan9353_write_i2c_reg(LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "write failed", err);

    // The completion of the write cycle is indicated by the clearing of the
    // CSR_BUSY bit
    do
    {
        // Read SWITCH_CSR_CMD register
        PHY_CHECK(lan9353_read_i2c_reg(LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "read failed", err);

        // Poll CSR_BUSY bit
    } while ((value & LAN9353_SWITCH_CSR_CMD_BUSY) != 0);

    return ESP_OK;

err:
    return ESP_FAIL;
}

esp_err_t lan9353_read_switch_reg(uint16_t address, uint32_t *val)
{
    // Set up a read operation
    uint32_t value = LAN9353_SWITCH_CSR_CMD_BUSY | LAN9353_SWITCH_CSR_CMD_READ |
                     LAN9353_SWITCH_CSR_CMD_BE;

    // Set register address
    value |= address & LAN9353_SWITCH_CSR_CMD_ADDR;

    // To perform a read of an individual switch fabric register, the read cycle
    // must be initiated by performing a single write to the SWITCH_CSR_CMD
    // register
    PHY_CHECK(lan9353_write_i2c_reg(LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "write failed", err);

    // Valid data is available for reading when the CSR_BUSY bit is cleared
    do
    {
        // Read SWITCH_CSR_CMD register
        PHY_CHECK(lan9353_read_i2c_reg(LAN9353_SWITCH_CSR_CMD, &value, 1) == ESP_OK, "read failed", err);

        // Poll CSR_BUSY bit
    } while ((value & LAN9353_SWITCH_CSR_CMD_BUSY) != 0);

    // Read data from the SWITCH_CSR_DATA register
    return lan9353_read_i2c_reg(LAN9353_SWITCH_CSR_DATA, val, 1);

err:
    return ESP_FAIL;
}