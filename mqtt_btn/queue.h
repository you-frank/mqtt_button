#define MAX_QUE 16 // 실제 max갯수는 15개임. 16개로 하면 Empty, Full모두 EnPos==DePos인 상황이 되어버림. 때문에 max를 하나 작은걸로 해야만 함.   4,8,16,32중에 size 선택할것!
#define NO_EVENT 0

typedef struct {
  int8_t x;
  int8_t z;
}xz_mv;

static xz_mv myQue[MAX_QUE];
static int8_t enQPos = -1;
static int8_t deQPos = 0;
static uint8_t qCount=0;

uint8_t cntQue()
{
  return qCount;
}

xz_mv deQue()
{
  xz_mv ret={0,0};
  if( qCount==0 ){
    return ret; // 0x00 
  }
  qCount--;
  ret = myQue[deQPos++];
  if(deQPos == MAX_QUE)
    deQPos=0;
  d_printf("de:[%d], %d\n", qCount, ret.z);
  return ret;
}

bool enQue(xz_mv dt)
{
  if(qCount == MAX_QUE) {  // if Full?
    return false; 
  }

  if(enQPos == MAX_QUE-1){ // if End?
    enQPos = -1;
  }
  
  myQue[++enQPos]=dt;
  qCount++;
  d_printf("en:[%d], %d\n", qCount, dt.z);
  return true;
}


void emptyQue()
{
  enQPos = -1;
  deQPos = 0;
  qCount = 0;
}
