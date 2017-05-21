#include "Planner.h";

Planner::Planner(void)
{

  typedef struct
  {
    bool busy;
    int leftPosition;
    int rightPosition;
  } bufferRing;

  bufferRing bufferQueue[RING_BUFFER_SIZE];
  volatile unsigned char tail;
  volatile unsigned char head;
  volatile unsigned char count;
  
  init(true);
}

void Planner::init (const bool clearBuffer)
{
    if(clearBuffer){
      memset (bufferQueue, 0, sizeof (*bufferQueue));
    }
    tail = 0;
    head = 0;
    count = 0;
}

int Planner::isEmpty(void)
{
  return (0 == count);
}

int Planner::isFull(void)
{
  return (count >= RING_BUFFER_SIZE);
}

void Planner::next(void)
{
  if(count > 0 && tail < head){
    bufferQueue[tail].busy = false;
    tail = modulo_inc(tail, RING_BUFFER_SIZE);
    --count;
  }
}

Planner::bufferRing Planner::get()
{
  bufferRing c;
  if(count > 0 && tail < head && !bufferQueue[tail].busy){

    bufferQueue[tail].busy = true;
    c = bufferQueue[tail];
    //tail = modulo_inc(tail, RING_BUFFER_SIZE);
    //--count;
    
  }else{
    c.leftPosition = 0;
    c.rightPosition = 0;
    c.busy = false;
  }
  return c;
}

void Planner::put(int leftPosition, int rightPosition)
{
  if(count < RING_BUFFER_SIZE){
    bufferQueue[head].leftPosition = leftPosition;
    bufferQueue[head].rightPosition = rightPosition;
    bufferQueue[head].busy = false;
    
    head = modulo_inc(head, RING_BUFFER_SIZE);
    ++count;
  }
}

unsigned char Planner::modulo_inc(int operand, int modulo)
{
  operand++;
  if(operand < modulo){
    return operand;
  }else{
    return (operand % modulo);
  }
}

