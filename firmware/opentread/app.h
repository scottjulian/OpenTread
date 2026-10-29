// app.h - the loop orchestrator.
// Sole owner of every core object (PulseFilter, Metrics). The .ino only
// forwards setup()/loop() here.
#pragma once

void app_setup();
void app_loop();
