#include <stdint.h>
#include <stdlib.h>
typedef unsigned char uint8;
typedef unsigned short uint16;
#define MT9V03X_W 188
extern volatile int Search_Stop_Line;
extern volatile int White_Column[MT9V03X_W];
int Zebra_Flag=0;
void Zebra_Detect(void)
{
    int j=0;
    int count=0;
    //前面元素互斥各位自行补全
    if(Search_Stop_Line>=60)//视野很长
    {
        for(j=10;j<=MT9V03X_W-10;j++)
        {
            if(abs(White_Column[j]-White_Column[j+1])>=30)
            {
                count++;
            }
        }
    }
    if(count>=6)
    {
        Zebra_Flag=1;
    }
}
