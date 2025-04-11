#include <M5EPD.h>

void clear_epd(M5EPD_Canvas &canvas)
{
    canvas.clear();
    canvas.setCursor(0, 0);
}

void init_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_info, M5EPD_Canvas &canvas_device_interfaces, M5EPD_Canvas &canvas_data_frame)
{
    canvas_title.createCanvas(540, 100);
    canvas_info.createCanvas(540, 60);
    canvas_device_interfaces.createCanvas(540, 300);
    canvas_data_frame.createCanvas(540, 500);

    canvas_title.setTextSize(3);
    canvas_info.setTextSize(2);
    canvas_device_interfaces.setTextSize(2);
    canvas_data_frame.setTextSize(2);

    clear_epd(canvas_title);
    clear_epd(canvas_info);
    clear_epd(canvas_device_interfaces);
    clear_epd(canvas_data_frame);
}

void update_epd(M5EPD_Canvas &canvas_title, M5EPD_Canvas &canvas_info, M5EPD_Canvas &canvas_device_interfaces, M5EPD_Canvas &canvas_data_frame)
{
    canvas_title.pushCanvas(0, 0, UPDATE_MODE_DU4);
    canvas_info.pushCanvas(0, 100, UPDATE_MODE_DU4);
    canvas_device_interfaces.pushCanvas(0, 160, UPDATE_MODE_DU4);
    canvas_data_frame.pushCanvas(0, 460, UPDATE_MODE_DU4);
}
