# ESP32cam
## pixel_format  
I'm really sorry you find this so frustrating. I had no idea that "GRAYSCALE", "RGB565" or any other of the formats need explanation.
1. GRAYSCALE means values from 0 to 255 for each pixel
2. RGB565 means that a 16 bit value is per pixel and is comprised from 5 bits red, 6 bits green and 5 bits blue.
3. RGB888 means that there are 3 bytes per pixel, one for red, one for green and one for blue. Each vary between 0 and 255.
4. YUV means that each pixel has it's Y channel as a separate byte (0 to 255) and each two adjacent pixels share their U and V values.
6. JPEG means that the image is encoded into JPEG format either by the camera itself or in software.  

When you get a frame from the camera, it will contain the frame buffer in the format you selected.  
https://www.esp32.com/viewtopic.php?t=10405  

## jpeg decoder  
https://github.com/espressif/esp32-camera/blob/master/conversions/include/esp_jpg_decode.h 

# ESP32cam with shrinked opencv 
https://www.youtube.com/watch?v=7qPIRBY6C8c&ab_channel=ThatProject  
https://github.com/joachimBurket/esp32-opencv 

# Docker啟動指令  
cd esp32-opencv  
docker run --rm -v $PWD:/project:z -w /project -it espressif/idf  
## build opencv  
./esp32/scripts/build_opencv_for_esp32.sh /opt/esp/idf/tools/cmake/toolchain-esp32.cmake /project/esp32/examples/ttgo_demo/main  
## build program  
cd esp32/examples/ttgo_demo  
idf.py build  
