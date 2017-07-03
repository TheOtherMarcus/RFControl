#include "RFControl.h"

#ifndef RF_CONTROL_VARDUINO
#include "arduino_functions.h"
#else
#define byte uint8_t
#endif

// Scale down time by 4 to fit in 16 bit unsigned int
#define PULSE_LENGTH_DIVIDER 4

// Noise filter
#define MIN_MSG_LEN 16

// Max length of data pulse. Longer pulses are treated as sync.
#define MAX_PULSE_PERIODS 20

// Minimum signal period time for a proper message
#define MIN_PERIOD_TIME (120 / PULSE_LENGTH_DIVIDER)

// Remembers the time of the last interrupt
volatile unsigned int lastTime;

// Pulse period time of message being received
volatile unsigned int periodTime;

// Pulse counter for message being received
volatile byte streak;

// Buffer pointer where the next message will be stored
volatile byte writer;

// Buffer pointer for the message reader
volatile byte reader;

// Circular message buffer (256) and extended raw buffer
volatile unsigned int msgbuf[MAX_RECORDINGS];

// Interrupt Service Routine
void isr();

// RF GPIO IN
int interruptPin = -1;

// Latest pulse time
volatile unsigned int duration;

// Indicates the availability of a pulse time to read in raw mode
volatile bool new_duration = false;

unsigned int RFControl::getPulseLengthDivider() {
  return PULSE_LENGTH_DIVIDER;
}

void RFControl::startReceiving(int _interruptPin) {
  lastTime = hw_micros() / 4;
  periodTime = 0;
  writer = 0;
  reader = 0;
  streak = 0;
  
  if(interruptPin != -1) {
    hw_detachInterrupt(interruptPin);   
  }
  interruptPin = _interruptPin;
  hw_attachInterrupt(interruptPin, isr);
}

void RFControl::stopReceiving() {
  if(interruptPin != -1) {
    hw_detachInterrupt(interruptPin);   
  }
  interruptPin = -1;
}

bool RFControl::hasData() {
  return writer != reader;
}

/* Message capture writes to a circular buffer, but the RFControl API
   is not compatible with such a construction. getRaw() unfolds the
   message into a linear sequence in memory. The circular buffer uses
   256 words, but msgbuf is typically larger than that. If the message
   wraps around, the words beyond index 255 is used for the unfolded
   message.
   
   A captured message starts with a one word header containing the
   average period time. After that follows the number of periods for
   each pulse in the message. getRaw() removes the header and
   multiplies the pulse periods with the period time to covert to the
   correct RFControl API message format.
 */
void RFControl::getRaw(unsigned int **buffer, unsigned int* timings_size) {
  static unsigned int size;
  unsigned int start = reader;
  unsigned int max_size = MAX_RECORDINGS - reader;
  byte pos = 0;
  size = 0;
  if (reader != writer) {
    unsigned int pt = msgbuf[(byte)(reader + pos++)];
    while ((byte)(reader+pos) != writer && msgbuf[(byte)(reader+pos)] <= MAX_PULSE_PERIODS && size < max_size) {
      msgbuf[start + size++] = pt * msgbuf[(byte)(reader + pos++)];
    }
    if (size >= max_size) {
      // Unable to fit message from circular buffer in flat buffer. Drop message
      size = 0;
    }
    else if ((reader+pos) != writer) {
      // Include sync at end
      msgbuf[start + size++] = pt * msgbuf[(byte)(reader + pos++)];      
    }
  }
  *timings_size = size;
  *buffer = (unsigned int*)&msgbuf[start];
}

void RFControl::continueReceiving() {
  if (reader != writer) {
    // Go to next message
    reader++;
    while (reader != writer && msgbuf[reader] <= MAX_PULSE_PERIODS) {
      reader++;
    }
    if (reader != writer) {
      // Include sync at end
      reader++;
    }
  }
}

unsigned int RFControl::getLastDuration(){
  new_duration = false;
  return duration;
}

bool RFControl::existNewDuration(){
  return new_duration;
}

void isr()
{
  unsigned int now = hw_micros() / PULSE_LENGTH_DIVIDER;
  unsigned int pulseTime = now - lastTime;
  unsigned int periods = (pulseTime + periodTime/2) / periodTime;
  byte lowPulse = digitalRead(interruptPin + 2);

  lastTime = now;
  duration = pulseTime;
  new_duration = true;
  
  if (periods == 0) {
    // Noise, ignore message
    streak = 0;
  }
  if (streak > 0) {
    // Receive message
    byte index = (writer + streak++);
    if (index == reader) {
      // Reception buffer is full, drop message
      streak = 0;
    }
    else {
      msgbuf[index] = periods;
    }
  }

  if (lowPulse) {
    if (periodTime > MIN_PERIOD_TIME && periods > MAX_PULSE_PERIODS) {
      // Sync detected
      if (streak > MIN_MSG_LEN) {
        // Message complete
        msgbuf[writer] = periodTime;
        writer = (writer + streak);
      }
      // Start new message
      streak = 1;
    }
  }
  else {
    // high pulse
    if (periods > MAX_PULSE_PERIODS) {
      // Noise, ignore message
      streak = 0;
    }
    if (streak > 0) {
      if (periods == 1) {
        // Approximate average of single period high pulses in message
        periodTime = (periodTime*streak + 2*pulseTime) / (streak + 2);
      }
    }
    else {
      // Initiate search for new period time and sync
      periodTime = pulseTime;
    }
  }
}


bool RFControl::compressTimings(unsigned int buckets[8], unsigned int *timings, unsigned int timings_size) {
  for(int j = 0; j < 8; j++ ) {
    buckets[j] = 0;
  }
  unsigned long sums[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int counts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  //sort timings into buckets, handle max 8 different pulse length
  for(unsigned int i = 0; i < timings_size; i++) 
  {
    int j = 0;
    for(; j < 8; j++) {
      unsigned int refVal = buckets[j];
      unsigned int val = timings[i];
      //if bucket is empty
      if(refVal == 0) {
        //sort into bucket
        buckets[j] = val;
        timings[i] = j;
        sums[j] += val;
        counts[j]++;
        break;
      } else {
        //check if bucket fits:
        unsigned int delta = refVal/4 + refVal/8;
        if(refVal - delta < val && val < refVal + delta) {
          timings[i] = j;
          sums[j] += val;
          counts[j]++;
          break;
        }
      }
      //try next..
    }
    if(j == 8) {
      //we have not found a bucket for this timing, exit...
      return false;
    }
  }
  for(int j = 0; j < 8; j++) {
    if(counts[j] != 0) {
      buckets[j] = sums[j] / counts[j];
    }
  }
  return true;
}

bool RFControl::compressTimingsAndSortBuckets(unsigned int buckets[8], unsigned int *timings, unsigned int timings_size) {
  //clear buckets
  for(int j = 0; j < 8; j++ ) {
    buckets[j] = 0;
  }
  //define arrays too calc the average value from the buckets
  unsigned long sums[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int counts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  //sort timings into buckets, handle max 8 different pulse length
  for(unsigned int i = 0; i < timings_size; i++) 
  {
    int j = 0;
    //timings need only there to load
    unsigned int val = timings[i];
    for(; j < 8; j++) {
      unsigned int refVal = buckets[j];
      //if bucket is empty
      if(refVal == 0) {
        //sort into bucket
        buckets[j] = val;
        sums[j] += val;
        counts[j]++;
        break;
      } else {
        //check if bucket fits:
        //its allowed round about 37,5% diff
        unsigned int delta = refVal/4 + refVal/8;
        if(refVal - delta < val && val < refVal + delta) {
          sums[j] += val;
          counts[j]++;
          break;
        }
      }
      //try next..
    }
    if(j == 8) {
      //we have not found a bucket for this timing, exit...
      return false;
    }
  }
  //calc the average value from the buckets
  for(int j = 0; j < 8; j++) {
    if(counts[j] != 0) {
      buckets[j] = sums[j] / counts[j];
    }
  }
  //buckets are defined
  //lets scramble a little bit
  for(int i = 0; i < 8; i++) {
    for(int j = 0; j < 7; j++) {
      if(buckets[j] > buckets[j+1]){
        unsigned int temp = buckets[j];
        buckets[j] = buckets[j+1];
        buckets[j+1] = temp;
      }
    }
  }
  // now the buckets are ordered by size from low to high.
  // but the zero ist first. lets move this back
  // find first value
  int first = 0;
  for(int i = 0; i < 8; i++){
    if(buckets[i] != 0){
    first = i;
    break;
    }
  }
  //copy buckets to the start of the array
  int end = 8 - first;
  for(int i = 0; i < end; i++){
    buckets[i] = buckets[first];
    buckets[first] = 0;
    first++;
  }
  
  //and now we can assign the timings with the position_value from the buckets
  //pre calc ref values. save time
  unsigned int ref_Val_h[8];
  unsigned int ref_Val_l[8];
  for(int i = 0; i<8;i++) {
    unsigned int refVal = buckets[i];
    //check if bucket fits:
    unsigned int delta = refVal/4 + refVal/8;
    ref_Val_h[i] = refVal + delta;
    ref_Val_l[i] = refVal - delta;
  }
  for(unsigned int i = 0; i < timings_size; i++) 
  {
    unsigned int val = timings[i];
    for(int j = 0; j < 8; j++) {
      if(ref_Val_l[j] < val && val < ref_Val_h[j]) {
        timings[i] = j;
        break;
      }
    }
  }
  return true;
}

void listenBeforeTalk()
{
  // listen before talk
  unsigned long waited = 0;
  if(interruptPin != -1) {
      waited += 500;
      hw_delayMicroseconds(500); 
    while(streak > 0) {
      //wait till no rf message is in the air
      waited += 5;
      hw_delayMicroseconds(3); // 5 - some micros for other stuff
      // don't wait longer than 5sec
      if(waited > 5000000) {
        break;
      }
      // some delay between the message in air and the new message send
      // there could be additional repeats following so wait some more time
      if(streak == 0) {
        waited += periodTime * MAX_PULSE_PERIODS;
        hw_delayMicroseconds(periodTime * MAX_PULSE_PERIODS);
      }
    }
    // stop receiving while sending, this method preserves the recording state
    hw_detachInterrupt(interruptPin);   
  }
}

void afterTalk()
{
  // enable reciving again
  if(interruptPin != -1) {
    hw_attachInterrupt(interruptPin, isr);
  }
}


void RFControl::sendByCompressedTimings(int transmitterPin,unsigned long* buckets, char* compressTimings, unsigned int repeats) {
  listenBeforeTalk();
  unsigned int timings_size = strlen(compressTimings);
  hw_pinMode(transmitterPin, OUTPUT);
  for(unsigned int i = 0; i < repeats; i++) {
    hw_digitalWrite(transmitterPin, LOW);
    int state = LOW;
    for(unsigned int j = 0; j < timings_size; j++) {
      state = !state;
      hw_digitalWrite(transmitterPin, state);
      unsigned int index = compressTimings[j] - '0';
      hw_delayMicroseconds(buckets[index]);
    }
  }
  hw_digitalWrite(transmitterPin, LOW);
  afterTalk();
}


void RFControl::sendByTimings(int transmitterPin, unsigned int *timings, unsigned int timings_size, unsigned int repeats) {
  listenBeforeTalk();

  hw_pinMode(transmitterPin, OUTPUT);
  for(unsigned int i = 0; i < repeats; i++) {
    hw_digitalWrite(transmitterPin, LOW);
    int state = LOW;
    for(unsigned int j = 0; j < timings_size; j++) {
      state = !state;
      hw_digitalWrite(transmitterPin, state);
      hw_delayMicroseconds(timings[j]);
    }
  }
  hw_digitalWrite(transmitterPin, LOW);
  afterTalk();
}

