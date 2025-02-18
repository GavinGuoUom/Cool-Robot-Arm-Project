#include <xArmServoController.h>
#define PI 3.141592653
#define use_sim_numbers true
#include <arduino.h>
#include <SoftwareSerial.h>
#include <angle_class.hpp>


//object setup

#define rx D6
#define tx D7

Robot CRAP (tx, rx);



// Test positions/paths store

    unsigned int home[6] = {500,500,500,500,500,150};
					  
    unsigned int bow[6]  = {500,130,650,845,500,650};   
					 
    double Store[51][5] = {{0.54042, 0.33443, 0.65867, 0.60096, 0},
						  {0.57791, 0.38313, 0.63564, 0.55312, 0},
						  {0.61633, 0.4257, 0.61461, 0.51147, 0},
						  {0.65545, 0.46107, 0.59647, 0.477, 0},
						  {0.69511, 0.48817, 0.58217, 0.45072, 0},
						  {0.73512, 0.506, 0.57256, 0.43351, 0},
						  {0.77533, 0.51379, 0.5683, 0.42602, 0},
						  {0.81558, 0.51118, 0.56973, 0.42852, 0},
						  {0.85571, 0.49829, 0.57678, 0.4409, 0},
						  {0.89557, 0.47567, 0.58896, 0.46262, 0},
						  {0.93498, 0.44421, 0.60554, 0.4929, 0},
						  {0.97378, 0.40497, 0.6256, 0.53077, 0},
						  {1.0118, 0.35902, 0.64821, 0.57524, 0},
						  {1.0487, 0.30738, 0.6725, 0.62535, 0},
						  {1.0844, 0.25094, 0.69767, 0.68021, 0},
						  {1.1185, 0.19044, 0.72305, 0.73906, 0},
						  {1.1507, 0.12655, 0.74802, 0.80118, 0},
						  {1.1806, 0.059795, 0.77208, 0.86594, 0},
						  {1.2077, -0.0093251, 0.79478, 0.93275, 0},
						  {1.2315, -0.080369, 0.8157, 1.001, 0},
						  {1.2512, -0.1529, 0.8345, 1.0701, 0},
						  {1.2659, -0.22645, 0.85087, 1.1395, 0},
						  {1.2745, -0.30048, 0.86452, 1.2083, 0},
						  {1.2758, -0.37427, 0.8752, 1.2758, 0},
						  {1.2679, -0.44695, 0.88274, 1.3412, 0},
						  {1.249, -0.51738, 0.88715, 1.4033, 0},
						  {1.2169, -0.58406, 0.88864, 1.4607, 0},
						  {1.1692, -0.64508, 0.88766, 1.512, 0},
						  {1.1041, -0.69808, 0.88499, 1.5554, 0},
						  {1.0212, -0.7403, 0.88163, 1.5892, 0},
						  {0.92253, -0.76894, 0.87872, 1.6117, 0},
						  {0.81339, -0.78168, 0.87725, 1.6216, 0},
						  {0.702, -0.7774, 0.87776, 1.6182, 0},
						  {0.5972, -0.75652, 0.88011, 1.6019, 0},
						  {0.50595, -0.72087, 0.88358, 1.5734, 0},
						  {0.43189, -0.6731, 0.88706, 1.5344, 0},
						  {0.37568, -0.61604, 0.88947, 1.4865, 0},
						  {0.33602, -0.55225, 0.88991, 1.4315, 0},
						  {0.31079, -0.48389, 0.88777, 1.371, 0},
						  {0.2977, -0.41262, 0.8827, 1.3065, 0},
						  {0.29462, -0.33972, 0.87457, 1.2392, 0},
						  {0.29974, -0.26615, 0.86339, 1.1701, 0},
						  {0.31158, -0.19265, 0.84926, 1.1001, 0},
						  {0.3289, -0.1198, 0.8324, 1.0302, 0},
						  {0.35072, -0.048122, 0.81313, 0.96082, 0},
						  {0.37625, 0.021903, 0.79181, 0.89268, 0},
						  {0.40483, 0.089823, 0.76885, 0.82633, 0},
						  {0.43594, 0.15517, 0.74469, 0.76234, 0},
						  {0.46915, 0.21742, 0.71981, 0.70133, 0},
						  {0.50408, 0.27598, 0.69477, 0.64397, 0},
						  {0.54042, 0.33015, 0.67019, 0.59102, 0}};


void setup(){
    Serial.begin(9600);
    CRAP.comms_start();
    delay(5000);
    Serial.print("working");

    // lil' bow test
    /*
      CRAP.set_positions(home, 3000);
      delay(1000);
      CRAP.set_positions(bow, 2000);
      delay(1000);
      CRAP.set_positions(home, 2000);
      delay(1000);
    */
    delay(3000);


    //set angles testing
    /*
        for (int i = 0; i < 15; i++) { //stored angles testing

            //setting angles

            Serial.print(i, DEC);
            double desired[5] = {Store[i][0],Store[i][1],Store[i][2],Store[i][3], Store[i][4]};
            
            int x;
            if(i==1) x =3000 else x = 150;

            CRAP.set_angles(desired[5], x);
            // delay(100); 


            //getting angles

            double angles[] = {1,2,3,4,5};
            CRAP.get_angles(angles);

            for (int j = 0; j < 5; j++) { // printing results - uncomment required blocks

                
                // Serial.print("\nServo ");
                // Serial.print(j+1, DEC);
                // Serial.print(" input angle: ");
                // Serial.print(Store[i][j]);
                
                // Serial.print("\nServo ");
                // Serial.print(j+1, DEC);
                // Serial.print(" output angle: ");
                // Serial.print(angles[j]);
                
                // Serial.print("\nServo ");
                // Serial.print(j+1, DEC);
                // Serial.print(" angle error: ");
                // Serial.print(angles[j]-Store[i][j]);
                
            }
            Serial.println();
        }
    */


}


void loop(){
    //Serial.print("working");

        //Read angles at 2 sec intervals

    // /*
        double angles[] = {1,2,3,4,5};
        CRAP.get_angles(angles);
        Serial.println();

        for (int j = 0; j < 5; j++) {

            Serial.print("\nServo ");
            Serial.print(j+1, DEC);
            Serial.print(" angle: ");
            Serial.print(angles[j]);
        }

        delay(2000);
    // */
    //get positions
    /*
    uint positions[] = [1,2,3,4,5];
    CRAP.get_positions(positions);
    Serial.println();

        for (int j = 0; j < 5; j++) {

            Serial.print("\nServo ");
            Serial.print(j+1, DEC);
            Serial.print(" angle: ");
            Serial.print(angles[j]);
        }

        delay(2000);
    */
   //delay(200);
}