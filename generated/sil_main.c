/* Harness SIL (gerado): le linhas "theta omega i r hand rearm collab", imprime "u sup tid" em hexfloat. */
#include <stdio.h>
#include "aurora_joint_ctrl.h"
int main(void)
{
    aa_joint_t c; double th, w, i, r; int h, ra, co;
    aa_joint_init(&c);
    while (scanf("%lf %lf %lf %lf %d %d %d", &th, &w, &i, &r, &h, &ra, &co) == 7) {
        float u = aa_joint_step(&c, (float)th, (float)w, (float)i, (float)r, h, ra, co);
        printf("%.9g %d %d\n", (double)u, (int)c.sup, (int)c.last_tid);
    }
    return 0;
}
