import numpy as np
class ConvertDexGenToleap():
    def __init__(self):
        self.gen_dex = np.zeros(25)
        self.translation = np.zeros(3)
        self.rotation = np.zeros(6)
        self.leap_hand[0:4] = np.zeros(4)    # Thumb
        self.leap_hand[4:8] = np.zeros(4)    # Index
        self.leap_hand[8:12] = np.zeros(4)   # Middle
        self.leap_hand[12:16] = np.zeros(4)  # Ring

    def convert(self, gen_dex):
        #Convert the dexterous grasp to leap hand joints angles
        self.gen_dex = np.array(gen_dex).flatten()
        self.translation = self.gen_dex[0:3]
        self.rotation = self.gen_dex[3:9]
        self.leap_hand[0:4] = self.gen_dex[9:13]    # Thumb
        self.leap_hand[4:8] = self.gen_dex[13:17]   # Index
        self.leap_hand[8:12] = self.gen_dex[17:21]  # Middle
        self.leap_hand[12:16] = self.gen_dex[21:25] # Ring
        return self.leap_hand