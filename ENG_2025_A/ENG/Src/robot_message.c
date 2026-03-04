#include "robot_message.h"

nuc_receive_data_t nuc_receive_data;
nuc_transmit_data_t nuc_transmit_data;
cmd_id_queue_t cmd_id_queue;
uint8_t nuc_pressed_if;
extern uint8_t ShiJue_mode_ctrl;
extern uint8_t t_shijue_reset;
int action_Num = 0;
uint8_t t_shijue = 0;

/*************************** SEND ********************************/

void data_update(uint8_t cmd_id)
{
    switch (cmd_id) {
        default:
            break;
    }
}

void send_data_to_nuc() //(uint8_t cmd_id)
{
    uint8_t len                            = 5;
    usb_cdc_data.usb_cdc_send_buf[0]       = 0xAA;
    usb_cdc_data.usb_cdc_send_buf[2]       = 0x0D;
    usb_cdc_data.usb_cdc_send_buf[3]       = ShiJue_mode_ctrl;
    usb_cdc_data.usb_cdc_send_buf[1]       = (len);
    usb_cdc_data.usb_cdc_send_buf[len - 1] = CRC_Calculation(usb_cdc_data.usb_cdc_send_buf, len - 1);
    usb_data_send(usb_cdc_data.usb_cdc_send_buf, len);
    if (ShiJue_mode_ctrl == 1) ShiJue_mode_ctrl = 0;
	if ((ShiJue_mode_ctrl == 2)&&(t_shijue_reset != 0)) t_shijue_reset--;
	if ((ShiJue_mode_ctrl == 2)&&(t_shijue_reset == 0)) ShiJue_mode_ctrl = 0;
}

/*************************** SEND ********************************/

/*************************** RECV ********************************/
/*head 0xAA
data {
   8 id = 0xFF 
   8 LENGTH = ?
   2*32 state[2] = {plan,execute}
   16 total = ?
   16 now_num = ?
    joint_plan[7]
}
tail crc8
*/
void Nuc_data_unpacked()
{
    if (usb_cdc_data.usb_cdc_rx_buf[0] == 0xAA && usb_cdc_data.usb_cdc_rx_buf[2] == 0xFF) {
		// uint16_t _length = usb_cdc_data.usb_cdc_rx_buf[2];
		uint16_t this_num;
		u8_to_int32_t(&(usb_cdc_data.usb_cdc_rx_buf[3]),&(nuc_receive_data.state[0]));
		u8_to_int32_t(&(usb_cdc_data.usb_cdc_rx_buf[7]),&(nuc_receive_data.state[1]));
		u8_to_uint16_t(&(usb_cdc_data.usb_cdc_rx_buf[11]),&(nuc_receive_data.total_num));
		u8_to_uint16_t(&(usb_cdc_data.usb_cdc_rx_buf[13]),&(this_num));
		if(nuc_receive_data.state[0]==1)
		{
			uint8_t t[4];
			for (int i = 0; i < 7; i++) 
			{
				for (int j = 0; j < 4; j++) {
					t[j] = usb_cdc_data.usb_cdc_rx_buf[15 + i * 4 + j];
				}
				if (i == 0) u8_to_float(t, &nuc_receive_data.action_data[this_num].len);
				if (i == 1) u8_to_float(t, &nuc_receive_data.action_data[this_num].yaw1);
				if (i == 2) u8_to_float(t, &nuc_receive_data.action_data[this_num].pitch1);
				if (i == 3) u8_to_float(t, &nuc_receive_data.action_data[this_num].yaw2);
				if (i == 4) u8_to_float(t, &nuc_receive_data.action_data[this_num].roll1);
				if (i == 5) u8_to_float(t, &nuc_receive_data.action_data[this_num].pitch2);
				if (i == 6) u8_to_float(t, &nuc_receive_data.action_data[this_num].roll2);
				if (action_Num < usb_cdc_data.usb_cdc_rx_buf[2]) action_Num = this_num;
			}
        }
		else 
		{
			nuc_receive_data.action_data[this_num].len = 0;
			nuc_receive_data.action_data[this_num].yaw1 = 0;
			nuc_receive_data.action_data[this_num].pitch1 = 0;
			nuc_receive_data.action_data[this_num].yaw2 = 0;
			nuc_receive_data.action_data[this_num].roll1 = 0;
			nuc_receive_data.action_data[this_num].pitch2 = 0;
			nuc_receive_data.action_data[this_num].roll2 = 0;
		}
		if(nuc_receive_data.state[0]==1) t_shijue = 1;
		else if((nuc_receive_data.state[0]==-6)&&(nuc_receive_data.state[0]==-2)) t_shijue = 2;
		else t_shijue = 0;
    }
    usb_cdc_data.usb_cdc_rx_flag = 0;
}

/*************************** RECV ********************************/

void float_to_u8(float *float_in, uint8_t *u8_out)
{
    uint8_t farray[4];
    *(float *)farray = *float_in;
    u8_out[3]        = farray[3];
    u8_out[2]        = farray[2];
    u8_out[1]        = farray[1];
    u8_out[0]        = farray[0];
}

void u8_to_float(uint8_t *datain, float *dataout)
{
    *dataout = *(float *)datain;
}

void u8_to_int32_t(uint8_t *datain, int32_t *dataout)
{
    *dataout = *(int32_t *)datain;
}

void u8_to_uint16_t(uint8_t *datain, uint16_t *dataout)
{
    *dataout = *(uint16_t *)datain;
}

void cmd_id_task_create(uint8_t cmd_id, uint16_t freq)
{
    cmd_id_queue.cmd_id_queue[cmd_id_queue.total_num] = cmd_id;
    cmd_id_queue.cmd_id_frq[cmd_id_queue.total_num]   = 1000 / (freq % 1000);
    cmd_id_queue.total_num += 1;
}

void cmd_id_queue_handle()
{
    send_data_to_nuc();
}

void cmd_id_init()
{
    cmd_id_queue.total_num = 0;
    cmd_id_queue.now_pos   = 0;
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len)
{
    uint8_t crc = 0xff;
    while (len--) {
        crc = CRC08_Table[crc ^ *ptr++];
    }
    return crc;
}

// uint8_t num_in_list_dect(uint8_t dect_id,uint8_t *dect_list,uint8_t lengths)
//{
//     uint8_t re=0;
//     for(int i=0;i<lengths;i++){
//         if(dect_list[i]==dect_id){
//             re=1;
//         }
//     }
//     return re;
// }
