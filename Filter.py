import math
class Filter:
    def __init__(self):
        self.pre_trans_x = None
        self.pre_trans_y = None
        self.pre_trans_z = None
        self.pre_angx = 0.00
        self.pre_angy = 0.00
    def update(self, tvecs,rvecs) -> bool:
        angy=rvecs[0][0][1]/math.pi*180
        angx=rvecs[0][0][0]/math.pi*180
        #angz=rvecs[0][0][2]/math.pi*180
        trans_x, trans_y, trans_z = tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2]
        is_mark_move = False
        is_mark_move_2 = False
        if self.pre_trans_x is not None:
            #if abs(self.pre_angx - angx) > 2.0 or abs(self.pre_angy - angy) > 2.0:
                if abs(self.pre_trans_x - trans_x) > 0.01 or abs(self.pre_trans_y - trans_y) > 0.02 or abs(self.pre_trans_z - trans_z) > 0.025:
                    dis_x = abs(self.pre_trans_x - trans_x)
                    dis_y = abs(self.pre_trans_y - trans_y)
                    dis_z = abs(self.pre_trans_z - trans_z)
                # if dis_x > 0.001:
                #     print('dis_x', dis_x)
                # if dis_y > 0.001:
                #     print("dis_y", dis_y)
                # if dis_z > 0.001:
                #     print("dis_z", dis_z)
                    is_mark_move = True
        if abs(self.pre_angx - angx) > 3.0 or abs(self.pre_angy - angy) > 3.0:
            is_mark_move_2 = True
            self.pre_angx,self.pre_angy = angx ,angy
        self.pre_trans_x, self.pre_trans_y, self.pre_trans_z = trans_x, trans_y, trans_z
        
        is_mark_move_3 = is_mark_move or is_mark_move_2
        return is_mark_move_3
