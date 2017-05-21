#include "Arduino.h"
#include "Configuration.h"

class Planner
{
  public:
    typedef struct
    {
      bool busy;
      int leftPosition;
      int rightPosition;
    } bufferRing;
    
    Planner(void);

    void        init    (const bool clearBuffer);
    int         isEmpty (void);
    int         isFull  (void);
    bufferRing  get     (void);
    void        next    (void);
    void        put     (int leftPosition, int rightPosition);

  //private:
    bufferRing bufferQueue[RING_BUFFER_SIZE];
    volatile unsigned char tail;
    volatile unsigned char head;
    volatile unsigned char count;
    unsigned char modulo_inc(int operand, int modulo);  
};


