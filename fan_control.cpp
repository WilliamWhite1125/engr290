TCCR0A|=(1<<COM0A1); // non-inverted pin operation
TCCR0A|=(1<<WGM00); // PWM, Phase Correct
OCR0A=0; // D=0, i.e. the fan is OFF. OCRA=255 will turn the fan ON at 100% of its power.
TCCR0B|=((1<<CS01)|(1<<CS00)); // Prescaler=64. Start the timer.
