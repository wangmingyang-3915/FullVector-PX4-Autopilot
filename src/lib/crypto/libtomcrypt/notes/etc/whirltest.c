#include <stdio.h>

int main(void)
{
   char buf[4096];
   int x;
   
   while (fgets(buf, sizeof(buf)-2, stdin) != NULL) {
        for (x = 0; x < 128; ) {
            printf("0x%c%c, ", buf[x], buf[x+1]);
            if (!((x += 2) & 31)) printf("\n");
        }
   }
}


/* ref:         HEAD -> main */
/* git commit:  684efeb5120df0df20fb56771cda3d310858777d */
/* commit time: 2026-06-07 22:27:16 +0800 */
