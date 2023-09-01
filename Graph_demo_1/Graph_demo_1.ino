// Demonstrate graph widget functions with a single trace instance
// One trace can be drawn at a time with one trace instance

// Requires widget library here:
// https://github.com/Bodmer/TFT_eWidget

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

#include <TFT_eWidget.h>               // Widget library

GraphWidget gr = GraphWidget(&tft);    // Graph widget gr instance with pointer to tft
TraceWidget tr = TraceWidget(&gr);     // Graph trace tr with pointer to gr

float gxLow  = 0;
float gxHigh = 0;
float gyLow  = 0;
float gyHigh = 0;
float peak = 0;
void draw_amp_graph(float peak){
  gxLow  = 0.0;
  gxHigh = len(array);
  gyLow = -1*peak
  gyHigh = peak;

  // Graph area is 320 pixels wide,30 pixels high, dark grey background
  gr.createGraph(320, 30, tft.color565(1, 3, 5));

  gr.setGraphScale(gxLow, gxHigh, gyLow, gxHigh);
  //draws graph starting at top left corner
  gr.drawGraph(0, 85);
      tr.startTrace(TFT_RED);
  // Create a new plot point every 10ms
  if (millis() - plotTime >= 10) {
    plotTime = millis();

    // Add a plot, first point in a trace will be a single pixel (if within graph area)
    while(gx < windowBuffer.size())
    tr.addPoint(gx, windowBuffer[gx]);
    gx += 1;
    gy += delta;

  
  }
}

void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Graph area is 200 pixels wide, 150 pixels high, dark grey background
  gr.createGraph(320, 30, tft.color565(1, 3, 5));

  // x scale units is from 0 to 100, y scale units is -512 to 512
  gr.setGraphScale(gxLow, gxHigh, gyLow, peak);

  // X grid starts at 0 with lines every 20 x-scale units
  // Y grid starts at -512 with lines every 64 y-scale units
  // blue grid
  // gr.setGraphGrid(gxLow, 20.0, gyLow, 64.0, TFT_BLUE);

  // Draw empty graph, top left corner at pixel coordinate 40,10 on TFT
  gr.drawGraph(0, 60);

  // Start a trace with using red, trace points are in x and y scale units
  // In this example a horizontal line is drawn
  tr.startTrace(TFT_RED);
  // Add a trace point at 0.0,0.0 on graph
  tr.addPoint(0.0, 0.0);
  // Add another point at 100.0, 0.0 this will be joined via line to the last point added
  tr.addPoint(100.0, 0.0);

  // Start a new trace with using white
  tr.startTrace(TFT_WHITE);
}

void loop() {
  static uint32_t plotTime = millis();
  static float gx = 0.0, gy = 0.0;
  static float delta = 10.0;

  // Create a new plot point every 100ms
  if (millis() - plotTime >= 10) {
    plotTime = millis();

    // Add a plot, first point in a trace will be a single pixel (if within graph area)
    tr.addPoint(gx, gy);
    gx += 1.0;
    if (gy >  500.0) delta = -10.0;
    if (gy < -500.0) delta =  10.0;
    gy += delta;

    // If the end of the graph x ais is reached start a new trace at 0.0,0.0
    if (gx > gxHigh) {
      gx = 0.0;
      gy = 0.0;

      // Draw empty graph at 40,10 on display to clear old one
      gr.drawGraph(0, 60);
      // Start new trace
      tr.startTrace(TFT_GREEN);
    }
  }
}
