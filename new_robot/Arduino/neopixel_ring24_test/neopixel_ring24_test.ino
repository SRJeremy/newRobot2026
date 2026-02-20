#include <Adafruit_NeoPixel.h>


#define PIN 7
#define NUMPIXELS 24
#define BRIGHT 100

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GBR + NEO_KHZ800);

// Slightly different, this makes the rainbow equally distributed throughout the ring
void rainbowCycle(uint8_t wait) {
  // 5 cycles of all colors on wheel, you can adjust the number 
  // of cycles by changing the first value (e.g., 1*65536 for one cycle)
  for(long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for(int i=0; i<pixels.numPixels(); i++) { // For each pixel in the ring
      // Offset pixel hue by an amount proportional to its position in the ring
      int pixelHue = firstPixelHue + (i * 65536L / pixels.numPixels());
      // Set the pixel color using the HSV color wheel (HSV is better for rainbows)
      pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
    }
    pixels.show(); // Update the LED ring
    delay(wait);  // Pause for a moment
  }
}


void setup() {
  // put your setup code here, to run once:
  pixels.begin();
  pixels.setBrightness(50);
  for(int i = 0; i < NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(BRIGHT, BRIGHT, BRIGHT));
  }
  pixels.show();
  
}

void loop() {
  //rainbowCycle(5);
  delay(500);

}
