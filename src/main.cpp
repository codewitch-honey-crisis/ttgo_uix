#include <Arduino.h>

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_DATA0 19 /*!< for 1-line SPI, this also refereed as MOSI */
#define PIN_NUM_PCLK 18
#define PIN_NUM_CS 5
#define PIN_NUM_DC 16
#define PIN_NUM_RST 23
#define PIN_NUM_BK_LIGHT 4
#define LCD_HRES 135
#define LCD_VRES 240
#define PIN_BUTTON_A 35
#define PIN_BUTTON_B 0

#include <button.hpp>
#include <gfx.hpp>
#include <uix.hpp>
using namespace arduino;
using namespace gfx;
using namespace uix;
// downloaded from fontsquirrel.com and header generated with
// https://honeythecodewitch.com/gfx/generator
#include <bee_icon.hpp>
#include <fonts/Telegrama.hpp>
static const open_font& text_font = Telegrama;

template <typename PixelType, typename PaletteType = gfx::palette<PixelType, PixelType>>
class svg_box : public control<PixelType, PaletteType> {
   public:
    using type = svg_box;
    using base_type = control<PixelType, PaletteType>;
    using pixel_type = PixelType;
    using palette_type = PaletteType;
    using control_surface_type = typename base_type::control_surface_type;
    typedef void (*on_pressed_changed_callback_type)(bool pressed, void* state);

   private:
    gfx::rgba_pixel<32> m_background_color;
    gfx::svg_doc* m_svg;
    svg_box(const svg_box& rhs) = delete;
    svg_box& operator=(const svg_box& rhs) = delete;
    void do_move(svg_box& rhs) {
        do_move_control(rhs);
        m_background_color = rhs.m_background_color;
        m_svg = rhs.m_svg;
    }
   public:
    svg_box(svg_box&& rhs) {
        do_move(rhs);
    }
    svg_box& operator=(svg_box&& rhs) {
        do_move(rhs);
        return *this;
    }
    svg_doc* doc() const {
        return m_svg;
    }
    void doc(svg_doc* value) {
        if(value!=m_svg) {
            m_svg = value;
            this->invalidate();
        }
    }
    svg_box(invalidation_tracker& parent, const palette_type* palette = nullptr) : base_type(parent, palette) {
        using color_t = gfx::color<gfx::rgba_pixel<32>>;
        m_background_color = color_t::white;
    }
    gfx::rgba_pixel<32> background_color() const {
        return m_background_color;
    }
    void background_color(gfx::rgba_pixel<32> value) {
        m_background_color = value;
        this->invalidate();
    }
   
    virtual void on_paint(control_surface_type& destination, const srect16& clip) override {
        srect16 b = (srect16)this->dimensions().bounds();
        if(m_svg!=nullptr) {
            gfx::draw::svg(destination,this->bounds().dimensions().bounds(),*m_svg,m_svg->scale(b.dimensions()));
        }
        base_type::on_paint(destination, clip);
    }
};

// declare the format of the screen
using screen_t = screen<LCD_VRES, LCD_HRES, rgb_pixel<16>>;
// declare the control types to match the screen
using label_t = label<typename screen_t::pixel_type>;
using svg_box_t = svg_box<typename screen_t::pixel_type>;
using color_t = color<typename screen_t::pixel_type>;
// for access to RGB8888 colors which controls use
using color32_t = color<rgba_pixel<32>>;
// declare the TTGO buttons
using button_a_t = int_button<PIN_BUTTON_A, 10, true>;
using button_b_t = int_button<PIN_BUTTON_B, 10, true>;

// UIX allows you to use two buffers for maximum DMA efficiency
// you don't have to, but performance is significantly better
// declare 64KB across two buffers for transfer
constexpr static const int lcd_buffer_size = 32 * 1024;
uint8_t lcd_buffer1[lcd_buffer_size];
uint8_t lcd_buffer2[lcd_buffer_size];
// this is the handle from the esp panel api
esp_lcd_panel_handle_t lcd_handle;
// our svg doc for svg_box
svg_doc doc;
// the TTGO buttons
button_a_t button_a;
button_b_t button_b;
// the main screen
screen_t main_screen(sizeof(lcd_buffer1), lcd_buffer1, lcd_buffer2);
// the controls
label_t test_label(main_screen);
svg_box_t test_svg(main_screen);

// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    main_screen.set_flush_complete();
    return true;
}
// tell the lcd panel api to transfer data via DMA
static void uix_flush(point16 location, typename screen_t::bitmap_type& bmp, void* state) {
    int x1 = location.x, y1 = location.y, x2 = location.x + bmp.dimensions().width, y2 = location.y + bmp.dimensions().height;
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2, y2, bmp.begin());
}
// initialize the screen using the esp panel API
void lcd_panel_init() {
    pinMode(PIN_NUM_BK_LIGHT, OUTPUT);

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = PIN_NUM_PCLK;
    buscfg.mosi_io_num = PIN_NUM_DATA0;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = PIN_NUM_DC,
    io_config.cs_gpio_num = PIN_NUM_CS,
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = PIN_NUM_RST;
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    digitalWrite(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, true);
    esp_lcd_panel_set_gap(lcd_handle, 40, 52);
    esp_lcd_panel_mirror(lcd_handle, false, true);
    esp_lcd_panel_invert_color(lcd_handle, true);
    // Turn on the screen
    esp_lcd_panel_disp_off(lcd_handle, false);
    // Turn on backlight (Different LCD screens may need different levels)
    digitalWrite(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
}
// initialize the screen and controls
void screen_init() {
    test_label.bounds(srect16(spoint16(10, 10), ssize16(200, 60)));
    test_label.text_color(color32_t::blue);
    test_label.text_open_font(&text_font);
    test_label.text_line_height(50);
    test_label.text_justify(uix_justify::center);
    test_label.round_ratio(NAN);
    test_label.padding({8, 8});
    test_label.text("Hello!");
    // make the backcolor transparent
    auto bg = color32_t::black;
    bg.channel<channel_name::A>(0);
    test_label.background_color(bg);
    // and the border
    test_label.border_color(bg);

    test_svg.bounds(srect16(spoint16(10, 70), ssize16(200, 60)));
    gfx_result res = svg_doc::read(&bee_icon, &doc);
    if (gfx_result::success != res) {
        Serial.printf("Error reading SVG: %d", (int)res);
    }
    test_svg.doc(&doc);
    main_screen.background_color(color_t::white);
    main_screen.register_control(test_label);
    main_screen.register_control(test_svg);
    main_screen.on_flush_callback(uix_flush);
}
// set up the hardware
void setup() {
    Serial.begin(115200);
    Serial.write(bee_icon_data, sizeof(bee_icon_data));
    Serial.println();
    lcd_panel_init();
    screen_init();
    button_a.initialize();
    button_b.initialize();
    button_a.on_pressed_changed([](bool pressed, void* state) {
        Serial.println("button_a");
        if (pressed) {
            test_label.text_color(color32_t::red);
        } else {
            test_label.text_color(color32_t::blue);
        }
    });
    button_b.on_pressed_changed([](bool pressed, void* state) {
        Serial.println("button_b");
        if (pressed) {
            main_screen.background_color(color_t::black);
        } else {
            main_screen.background_color(color_t::white);
        }
    });
    
}
// keep our stuff up to date and responsive
void loop() {
     button_a.update();
     button_b.update();
     main_screen.update();
}