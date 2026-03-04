//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_1_0;

ui_interface_line_t *ui_g_1_qianzuo = (ui_interface_line_t*)&(ui_g_1_0.data[0]);
ui_interface_line_t *ui_g_1_qianyou = (ui_interface_line_t*)&(ui_g_1_0.data[1]);
ui_interface_line_t *ui_g_1_jingjie = (ui_interface_line_t*)&(ui_g_1_0.data[2]);

void _ui_init_g_1_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_0.data[i].figure_name[0] = 0;
        ui_g_1_0.data[i].figure_name[1] = 0;
        ui_g_1_0.data[i].figure_name[2] = i + 0;
        ui_g_1_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_1_0.data[i].operate_type = 0;
    }

    ui_g_1_qianzuo->figure_type = 0;
    ui_g_1_qianzuo->operate_type = 1;
    ui_g_1_qianzuo->layer = 0;
    ui_g_1_qianzuo->color = 8;
    ui_g_1_qianzuo->start_x = 868;
    ui_g_1_qianzuo->start_y = 0;
    ui_g_1_qianzuo->width = 7;
    ui_g_1_qianzuo->end_x = 915;
    ui_g_1_qianzuo->end_y = 621;

    ui_g_1_qianyou->figure_type = 0;
    ui_g_1_qianyou->operate_type = 1;
    ui_g_1_qianyou->layer = 0;
    ui_g_1_qianyou->color = 8;
    ui_g_1_qianyou->start_x = 1376;
    ui_g_1_qianyou->start_y = 3;
    ui_g_1_qianyou->width = 7;
    ui_g_1_qianyou->end_x = 1130;
    ui_g_1_qianyou->end_y = 616;

    ui_g_1_jingjie->figure_type = 0;
    ui_g_1_jingjie->operate_type = 1;
    ui_g_1_jingjie->layer = 0;
    ui_g_1_jingjie->color = 8;
    ui_g_1_jingjie->start_x = 24;
    ui_g_1_jingjie->start_y = 200;
    ui_g_1_jingjie->width = 10;
    ui_g_1_jingjie->end_x = 1904;
    ui_g_1_jingjie->end_y = 200;


    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_update_g_1_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_remove_g_1_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}


void ui_init_g_1() {
    _ui_init_g_1_0();
}

void ui_update_g_1() {
    _ui_update_g_1_0();
}

void ui_remove_g_1() {
    _ui_remove_g_1_0();
}

ui_2_frame_t ui_g_2_0;

ui_interface_line_t *ui_g_2_houzuo = (ui_interface_line_t*)&(ui_g_2_0.data[0]);
ui_interface_line_t *ui_g_2_houyou = (ui_interface_line_t*)&(ui_g_2_0.data[1]);

void _ui_init_g_2_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_2_0.data[i].figure_name[0] = 0;
        ui_g_2_0.data[i].figure_name[1] = 1;
        ui_g_2_0.data[i].figure_name[2] = i + 0;
        ui_g_2_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_2_0.data[i].operate_type = 0;
    }

    ui_g_2_houzuo->figure_type = 0;
    ui_g_2_houzuo->operate_type = 1;
    ui_g_2_houzuo->layer = 0;
    ui_g_2_houzuo->color = 8;
    ui_g_2_houzuo->start_x = 414;
    ui_g_2_houzuo->start_y = 2;
    ui_g_2_houzuo->width = 7;
    ui_g_2_houzuo->end_x = 839;
    ui_g_2_houzuo->end_y = 472;

    ui_g_2_houyou->figure_type = 0;
    ui_g_2_houyou->operate_type = 1;
    ui_g_2_houyou->layer = 0;
    ui_g_2_houyou->color = 8;
    ui_g_2_houyou->start_x = 1082;
    ui_g_2_houyou->start_y = 1;
    ui_g_2_houyou->width = 7;
    ui_g_2_houyou->end_x = 950;
    ui_g_2_houyou->end_y = 470;


    ui_proc_2_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_update_g_2_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_2_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_remove_g_2_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_2_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}


void ui_init_g_2() {
    _ui_init_g_2_0();
}

void ui_update_g_2() {
    _ui_update_g_2_0();
}

void ui_remove_g_2() {
    _ui_remove_g_2_0();
}

ui_1_frame_t ui_g_4_0;

ui_interface_round_t *ui_g_4_gk = (ui_interface_round_t*)&(ui_g_4_0.data[0]);

void _ui_init_g_4_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_4_0.data[i].figure_name[0] = 0;
        ui_g_4_0.data[i].figure_name[1] = 2;
        ui_g_4_0.data[i].figure_name[2] = i + 0;
        ui_g_4_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_4_0.data[i].operate_type = 0;
    }

    ui_g_4_gk->figure_type = 2;
    ui_g_4_gk->operate_type = 1;
    ui_g_4_gk->layer = 0;
    ui_g_4_gk->color = 2;
    ui_g_4_gk->start_x = 937;
    ui_g_4_gk->start_y = 417;
    ui_g_4_gk->width = 4;
    ui_g_4_gk->r = 16;


    ui_proc_1_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

void _ui_update_g_4_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_4_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}

void _ui_remove_g_4_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_4_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_4_0, sizeof(ui_g_4_0));
}


void ui_init_g_4() {
    _ui_init_g_4_0();
}

void ui_update_g_4() {
    _ui_update_g_4_0();
}

void ui_remove_g_4() {
    _ui_remove_g_4_0();
}

ui_2_frame_t ui_g_Ungroup_0;

ui_interface_line_t *ui_g_Ungroup_gjleft = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_gjright = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[1]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 3;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_gjleft->figure_type = 0;
    ui_g_Ungroup_gjleft->operate_type = 1;
    ui_g_Ungroup_gjleft->layer = 0;
    ui_g_Ungroup_gjleft->color = 2;
    ui_g_Ungroup_gjleft->start_x = 1191;
    ui_g_Ungroup_gjleft->start_y = 318;
    ui_g_Ungroup_gjleft->width = 5;
    ui_g_Ungroup_gjleft->end_x = 1240;
    ui_g_Ungroup_gjleft->end_y = 620;

    ui_g_Ungroup_gjright->figure_type = 0;
    ui_g_Ungroup_gjright->operate_type = 1;
    ui_g_Ungroup_gjright->layer = 0;
    ui_g_Ungroup_gjright->color = 2;
    ui_g_Ungroup_gjright->start_x = 1138;
    ui_g_Ungroup_gjright->start_y = 384;
    ui_g_Ungroup_gjright->width = 5;
    ui_g_Ungroup_gjright->end_x = 1440;
    ui_g_Ungroup_gjright->end_y = 410;


    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}


void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
}

ui_1_frame_t ui_g_again_0;

ui_interface_round_t *ui_g_again_again = (ui_interface_round_t*)&(ui_g_again_0.data[0]);

void _ui_init_g_again_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_again_0.data[i].figure_name[0] = 0;
        ui_g_again_0.data[i].figure_name[1] = 4;
        ui_g_again_0.data[i].figure_name[2] = i + 0;
        ui_g_again_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_again_0.data[i].operate_type = 0;
    }

    ui_g_again_again->figure_type = 2;
    ui_g_again_again->operate_type = 1;
    ui_g_again_again->layer = 0;
    ui_g_again_again->color = 1;
    ui_g_again_again->start_x = 296;
    ui_g_again_again->start_y = 828;
    ui_g_again_again->width = 20;
    ui_g_again_again->r = 15;


    ui_proc_1_frame(&ui_g_again_0);
    SEND_MESSAGE((uint8_t *) &ui_g_again_0, sizeof(ui_g_again_0));
}

void _ui_update_g_again_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_again_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_again_0);
    SEND_MESSAGE((uint8_t *) &ui_g_again_0, sizeof(ui_g_again_0));
}

void _ui_remove_g_again_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_again_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_again_0);
    SEND_MESSAGE((uint8_t *) &ui_g_again_0, sizeof(ui_g_again_0));
}


void ui_init_g_again() {
    _ui_init_g_again_0();
}

void ui_update_g_again() {
    _ui_update_g_again_0();
}

void ui_remove_g_again() {
    _ui_remove_g_again_0();
}

ui_1_frame_t ui_g_close_0;

ui_interface_round_t *ui_g_close_close = (ui_interface_round_t*)&(ui_g_close_0.data[0]);

void _ui_init_g_close_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_close_0.data[i].figure_name[0] = 0;
        ui_g_close_0.data[i].figure_name[1] = 5;
        ui_g_close_0.data[i].figure_name[2] = i + 0;
        ui_g_close_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_close_0.data[i].operate_type = 0;
    }

    ui_g_close_close->figure_type = 2;
    ui_g_close_close->operate_type = 1;
    ui_g_close_close->layer = 0;
    ui_g_close_close->color = 7;
    ui_g_close_close->start_x = 298;
    ui_g_close_close->start_y = 742;
    ui_g_close_close->width = 20;
    ui_g_close_close->r = 15;


    ui_proc_1_frame(&ui_g_close_0);
    SEND_MESSAGE((uint8_t *) &ui_g_close_0, sizeof(ui_g_close_0));
}

void _ui_update_g_close_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_close_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_close_0);
    SEND_MESSAGE((uint8_t *) &ui_g_close_0, sizeof(ui_g_close_0));
}

void _ui_remove_g_close_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_close_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_close_0);
    SEND_MESSAGE((uint8_t *) &ui_g_close_0, sizeof(ui_g_close_0));
}


void ui_init_g_close() {
    _ui_init_g_close_0();
}

void ui_update_g_close() {
    _ui_update_g_close_0();
}

void ui_remove_g_close() {
    _ui_remove_g_close_0();
}

ui_1_frame_t ui_g_die_0;

ui_interface_round_t *ui_g_die_die = (ui_interface_round_t*)&(ui_g_die_0.data[0]);

void _ui_init_g_die_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_die_0.data[i].figure_name[0] = 0;
        ui_g_die_0.data[i].figure_name[1] = 6;
        ui_g_die_0.data[i].figure_name[2] = i + 0;
        ui_g_die_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_die_0.data[i].operate_type = 0;
    }

    ui_g_die_die->figure_type = 2;
    ui_g_die_die->operate_type = 1;
    ui_g_die_die->layer = 0;
    ui_g_die_die->color = 7;
    ui_g_die_die->start_x = 296;
    ui_g_die_die->start_y = 828;
    ui_g_die_die->width = 20;
    ui_g_die_die->r = 15;


    ui_proc_1_frame(&ui_g_die_0);
    SEND_MESSAGE((uint8_t *) &ui_g_die_0, sizeof(ui_g_die_0));
}

void _ui_update_g_die_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_die_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_die_0);
    SEND_MESSAGE((uint8_t *) &ui_g_die_0, sizeof(ui_g_die_0));
}

void _ui_remove_g_die_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_die_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_die_0);
    SEND_MESSAGE((uint8_t *) &ui_g_die_0, sizeof(ui_g_die_0));
}


void ui_init_g_die() {
    _ui_init_g_die_0();
}

void ui_update_g_die() {
    _ui_update_g_die_0();
}

void ui_remove_g_die() {
    _ui_remove_g_die_0();
}

ui_1_frame_t ui_g_ok_0;

ui_interface_round_t *ui_g_ok_ok = (ui_interface_round_t*)&(ui_g_ok_0.data[0]);

void _ui_init_g_ok_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_ok_0.data[i].figure_name[0] = 0;
        ui_g_ok_0.data[i].figure_name[1] = 7;
        ui_g_ok_0.data[i].figure_name[2] = i + 0;
        ui_g_ok_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_ok_0.data[i].operate_type = 0;
    }

    ui_g_ok_ok->figure_type = 2;
    ui_g_ok_ok->operate_type = 1;
    ui_g_ok_ok->layer = 0;
    ui_g_ok_ok->color = 2;
    ui_g_ok_ok->start_x = 296;
    ui_g_ok_ok->start_y = 828;
    ui_g_ok_ok->width = 20;
    ui_g_ok_ok->r = 15;


    ui_proc_1_frame(&ui_g_ok_0);
    SEND_MESSAGE((uint8_t *) &ui_g_ok_0, sizeof(ui_g_ok_0));
}

void _ui_update_g_ok_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_ok_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_ok_0);
    SEND_MESSAGE((uint8_t *) &ui_g_ok_0, sizeof(ui_g_ok_0));
}

void _ui_remove_g_ok_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_ok_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_ok_0);
    SEND_MESSAGE((uint8_t *) &ui_g_ok_0, sizeof(ui_g_ok_0));
}


void ui_init_g_ok() {
    _ui_init_g_ok_0();
}

void ui_update_g_ok() {
    _ui_update_g_ok_0();
}

void ui_remove_g_ok() {
    _ui_remove_g_ok_0();
}

ui_1_frame_t ui_g_open_0;

ui_interface_round_t *ui_g_open_open = (ui_interface_round_t*)&(ui_g_open_0.data[0]);

void _ui_init_g_open_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_open_0.data[i].figure_name[0] = 0;
        ui_g_open_0.data[i].figure_name[1] = 8;
        ui_g_open_0.data[i].figure_name[2] = i + 0;
        ui_g_open_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_open_0.data[i].operate_type = 0;
    }

    ui_g_open_open->figure_type = 2;
    ui_g_open_open->operate_type = 1;
    ui_g_open_open->layer = 0;
    ui_g_open_open->color = 2;
    ui_g_open_open->start_x = 298;
    ui_g_open_open->start_y = 742;
    ui_g_open_open->width = 20;
    ui_g_open_open->r = 15;


    ui_proc_1_frame(&ui_g_open_0);
    SEND_MESSAGE((uint8_t *) &ui_g_open_0, sizeof(ui_g_open_0));
}

void _ui_update_g_open_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_open_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_open_0);
    SEND_MESSAGE((uint8_t *) &ui_g_open_0, sizeof(ui_g_open_0));
}

void _ui_remove_g_open_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_open_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_open_0);
    SEND_MESSAGE((uint8_t *) &ui_g_open_0, sizeof(ui_g_open_0));
}


void ui_init_g_open() {
    _ui_init_g_open_0();
}

void ui_update_g_open() {
    _ui_update_g_open_0();
}

void ui_remove_g_open() {
    _ui_remove_g_open_0();
}


ui_string_frame_t ui_g_pumg_0;
ui_interface_string_t* ui_g_pumg_pumg = &(ui_g_pumg_0.option);

void _ui_init_g_pumg_0() {
    ui_g_pumg_0.option.figure_name[0] = 0;
    ui_g_pumg_0.option.figure_name[1] = 9;
    ui_g_pumg_0.option.figure_name[2] = 0;
    ui_g_pumg_0.option.operate_type = 1;

    ui_g_pumg_pumg->figure_type = 7;
    ui_g_pumg_pumg->operate_type = 1;
    ui_g_pumg_pumg->layer = 0;
    ui_g_pumg_pumg->color = 4;
    ui_g_pumg_pumg->start_x = 68;
    ui_g_pumg_pumg->start_y = 780;
    ui_g_pumg_pumg->width = 4;
    ui_g_pumg_pumg->font_size = 40;
    ui_g_pumg_pumg->str_length = 4;
    strcpy(ui_g_pumg_pumg->string, "PUMG");


    ui_proc_string_frame(&ui_g_pumg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_pumg_0, sizeof(ui_g_pumg_0));
}

void _ui_update_g_pumg_0() {
    ui_g_pumg_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_pumg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_pumg_0, sizeof(ui_g_pumg_0));
}

void _ui_remove_g_pumg_0() {
    ui_g_pumg_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_pumg_0);
    SEND_MESSAGE((uint8_t *) &ui_g_pumg_0, sizeof(ui_g_pumg_0));
}

void ui_init_g_pumg() {
    _ui_init_g_pumg_0();
}

void ui_update_g_pumg() {
    _ui_update_g_pumg_0();
}

void ui_remove_g_pumg() {
    _ui_remove_g_pumg_0();
}


ui_string_frame_t ui_g_shijue_0;
ui_interface_string_t* ui_g_shijue_shijue = &(ui_g_shijue_0.option);

void _ui_init_g_shijue_0() {
    ui_g_shijue_0.option.figure_name[0] = 0;
    ui_g_shijue_0.option.figure_name[1] = 10;
    ui_g_shijue_0.option.figure_name[2] = 0;
    ui_g_shijue_0.option.operate_type = 1;

    ui_g_shijue_shijue->figure_type = 7;
    ui_g_shijue_shijue->operate_type = 1;
    ui_g_shijue_shijue->layer = 0;
    ui_g_shijue_shijue->color = 6;
    ui_g_shijue_shijue->start_x = 13;
    ui_g_shijue_shijue->start_y = 867;
    ui_g_shijue_shijue->width = 4;
    ui_g_shijue_shijue->font_size = 40;
    ui_g_shijue_shijue->str_length = 6;
    strcpy(ui_g_shijue_shijue->string, "Vision");


    ui_proc_string_frame(&ui_g_shijue_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shijue_0, sizeof(ui_g_shijue_0));
}

void _ui_update_g_shijue_0() {
    ui_g_shijue_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_shijue_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shijue_0, sizeof(ui_g_shijue_0));
}

void _ui_remove_g_shijue_0() {
    ui_g_shijue_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_shijue_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shijue_0, sizeof(ui_g_shijue_0));
}

void ui_init_g_shijue() {
    _ui_init_g_shijue_0();
}

void ui_update_g_shijue() {
    _ui_update_g_shijue_0();
}

void ui_remove_g_shijue() {
    _ui_remove_g_shijue_0();
}

