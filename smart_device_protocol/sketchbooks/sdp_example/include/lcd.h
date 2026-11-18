#include <LovyanGFX.hpp>

#include "sdp/sdp_util.h"

void init_lcd(LGFX &lcd, LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2, LGFX_Sprite &sprite_3)
{
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(128);
    lcd.setColorDepth(24);
    lcd.fillScreen(0xFFFFFF);

    sprite_1.createSprite(300, 50);
    sprite_2.createSprite(300, 50);
    sprite_3.createSprite(300, 50);

    sprite_1.fillScreen(0xFFFFFF);
    sprite_1.setTextColor(0x000000);
    sprite_2.fillScreen(0xFFFFFF);
    sprite_2.setTextColor(0x000000);
    sprite_3.fillScreen(0xFFFFFF);
    sprite_3.setTextColor(0x000000);
}

void update_lcd(LGFX_Sprite &sprite_1, LGFX_Sprite &sprite_2, LGFX_Sprite &sprite_3)
{
    sprite_1.pushSprite(0, 0);
    sprite_2.pushSprite(0, 60);
    sprite_3.pushSprite(0, 120);
}

void clear_sprite(LGFX_Sprite &sprite)
{
    sprite.fillScreen(0xFFFFFF);
    sprite.setCursor(0, 0);
}

void print_sdp_body(LGFX_Sprite &sprite, const std::vector<SDPData> &body)
{
    std::string serialization_format = get_serialization_format(body);
    auto itr = body.begin();
    int index_sf = 0;
    while (itr != body.end())
    {
        switch (serialization_format[index_sf])
        {
        case 'i':
            sprite.printf("%d: %d\n", index_sf, std::get<int32_t>(*itr));
            break;
        case 'f':
            sprite.printf("%d: %f\n", index_sf, std::get<float>(*itr));
            break;
        case 's':
            sprite.printf("%d: %s\n", index_sf, std::get<std::string>(*itr).c_str());
            break;
        case 'S':
            sprite.printf("%d: %s\n", index_sf, std::get<std::string>(*itr).c_str());
            break;
        case '?':
        case 'b':
            sprite.printf("%d: %s\n", index_sf, std::get<bool>(*itr) ? "true" : "false");
            break;
        default:
            sprite.printf("%d: %s\n", index_sf, "unknown");
            break;
        }
        ++itr;
        ++index_sf;
    }
}