//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_POWER_CONTRAL_H
#define H7_PREPARED_DEVELOPMENT_POWER_CONTRAL_H
typedef enum {
    Ch_15V_1,
    Ch_15V_2,
    Ch_5V,
    ALL
} Power_Channal;

void Power_Contral(Power_Channal Open_ch);
#endif //H7_PREPARED_DEVELOPMENT_POWER_CONTRAL_H
