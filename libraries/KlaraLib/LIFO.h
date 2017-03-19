   
#define LIFO_SIZE 25
#define MAX_UNSIGNED_LONG 4294967295

class LIFO {
  volatile unsigned long buff[LIFO_SIZE] = {0};
  volatile byte cursor = 0;
  
  public:
  void push(unsigned long val) {
    buff[cursor] = val;
    cursor = (cursor+1)%LIFO_SIZE;
  }

  unsigned long getDiff() {
    byte c = cursor;
    unsigned long oldd = buff[c];
    unsigned long neww = buff[(c + LIFO_SIZE - 1) % LIFO_SIZE];
    return (0 == oldd or 0 == neww) ? MAX_UNSIGNED_LONG : (neww - oldd);
  }

};