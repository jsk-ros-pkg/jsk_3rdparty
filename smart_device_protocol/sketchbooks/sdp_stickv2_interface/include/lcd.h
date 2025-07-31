#include <LovyanGFX.hpp>

void clear_sprite(LGFX_Sprite &sprite)
{
    sprite.fillScreen(0xFFFFFF);
    sprite.setCursor(0, 0);
}

void init_lcd(LGFX &lcd, LGFX_Sprite &sprite_title, LGFX_Sprite &sprite_status, LGFX_Sprite &sprite_info)
{
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(128);
    lcd.setColorDepth(24);
    lcd.fillScreen(0xFFFFFF);

    sprite_title.createSprite(320, 60);
    sprite_status.createSprite(320, 60);
    sprite_info.createSprite(320, 120);

    sprite_title.setTextColor(0x000000);
    sprite_status.setTextColor(0x000000);
    sprite_info.setTextColor(0x000000);

    sprite_title.setTextSize(1.2, 1.2);
    sprite_status.setTextSize(1.0, 1.0);
    sprite_info.setTextSize(1.0, 1.0);

    clear_sprite(sprite_title);
    clear_sprite(sprite_status);
    clear_sprite(sprite_info);
}

void update_lcd(LGFX &lcd, LGFX_Sprite &sprite_title, LGFX_Sprite &sprite_status, LGFX_Sprite &sprite_info)
{
    sprite_title.pushSprite(0, 0);
    sprite_status.pushSprite(0, 60);
    sprite_info.pushSprite(0, 120);
}