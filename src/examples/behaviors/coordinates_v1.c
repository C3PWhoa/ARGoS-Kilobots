/*
   _____                    _ _             _                          __        ___           _       _
  / ____|                  | (_)           | |                        /_ |      / _ \         | |     | |
 | |     ___   ___  _ __ __| |_ _ __   __ _| |_ ___  ___      __   __  | |     | | | |    __ _| |_ __ | |__   __ _
 | |    / _ \ / _ \| '__/ _` | | '_ \ / _` | __/ _ \/ __|     \ \ / /  | |     | | | |   / _` | | '_ \| '_ \ / _` |
 | |___| (_) | (_) | | | (_| | | | | | (_| | ||  __/\__ \      \ V /   | |  _  | |_| |  | (_| | | |_) | | | | (_| |
  \_____\___/ \___/|_|  \__,_|_|_| |_|\__,_|\__\___||___/       \_(_)  |_| (_)  \___/    \__,_|_| .__/|_| |_|\__,_|
                                                                                                | |
                                                                                                |_|

Decentralized global coordinate system construction for kilobots
Example application: displaying message
Displayed Message: Hello World (for 100 robots in 10x10 formation)

code (c) by Michal Pluhacek
Version date: February 3rd, 2023

Paper citation: to be added

*/


#include <kilolib.h>
#include <stdio.h>
#include <stdlib.h>
//#include <math.h>
// !!!!!!!!!!!!!!!!!
#define ARGOS // comment before compiling for real robots
// !!!!!!!!!!!!!!!!!


// defining the message to display {x1,y1,x2,y2.........xn,yn}
int p1[]={1,10,1,8,1,7,1,9,1,6,2,8,3,10,3,8,3,9,3,7,3,6,2,5,2,4,2,3,2,1,3,1,2,2,5,5,5,4,5,2,5,3,5,1,6,1,8,2,8,3,8,4,9,5,10,4,10,3,10,2,9,1,6,10,6,9,6,7,6,6,6,8,7,6,8,6,8,8,7,8,7,10,8,10}; //HELLO 42
int p2[]={1,10,1,9,2,7,1,8,3,8,4,7,5,8,5,9,5,10,7,9,7,8,9,8,9,9,8,10,1,2,1,3,1,4,1,1,2,5,3,4,2,3,3,2,3,1,5,5,5,4,5,3,5,1,5,2,6,1,8,5,8,4,8,3,8,2,8,1,9,5,10,4,10,3,10,2,9,1,8,7,3,5,1,5}; // WORLD 42
int p3[]={1,10,1,8,1,7,1,9,1,6,2,8,3,10,3,8,3,9,3,7,3,6,2,5,2,4,2,3,2,1,3,1,2,2,5,5,5,4,5,2,5,3,5,1,6,1,8,2,8,3,8,4,9,5,10,4,10,3,10,2,9,1,6,10,6,9,6,7,6,6,6,8,7,6,8,6,8,8,7,8,7,10,8,10}; //HELLO 42
int p4[]={1,10,1,9,2,7,1,8,3,8,4,7,5,8,5,9,5,10,7,9,7,8,9,8,9,9,8,10,1,2,1,3,1,4,1,1,2,5,3,4,2,3,3,2,3,1,5,5,5,4,5,3,5,1,5,2,6,1,8,5,8,4,8,3,8,2,8,1,9,5,10,4,10,3,10,2,9,1,8,7,3,5,1,5}; // WORLD 42
int lengths[]={42,42,42,42}; // Number of coordinates to display particular step of the message
int letters=4; // Number of letters / steps to display


// variables:
message_t message;
int  new_message = 0;
int  sent_message = 0;
int MY_ID = 0;
int MY_ID2 = 0;
uint8_t MY_VALUE = 3;
uint8_t flag = 0;
uint8_t flag2 = 0;
uint8_t MY_STATE = 0;  // 0 - middle; 1 - border; 2 - corner;
uint8_t MY_STATE_value_min = 0;
uint8_t MY_STATE_value_max = 0;
uint32_t last_changed = 0;
uint32_t last_changed2 = 0;
int BlackList[90];
int blcounter=0;
int neighbor_ID[8];
int neighbor_DIST[8];
uint8_t phase=0;
int p=0;
int control_ID[255];
uint8_t received[9];
uint8_t sending[9];
uint8_t pom=0;
int distance = 255;
int min_distance = 255;
int neighbor_count = 0;
int active=1;
int x=0;
int y=0;
int im_origin=0;
uint8_t maxX=0;
uint8_t maxY=0;
int step=0;
uint8_t change_flag = 0;
int k;
uint8_t f=0;
uint8_t g=0;
uint8_t maxf=0;
uint8_t cornerf1=0;
uint8_t cornerf2=0;
uint8_t cornerf3=0;
uint8_t fcounter=3;
uint8_t x1=0;
uint8_t x2=0;
uint8_t x3=0;
uint8_t yy1=0;
uint8_t y2=0;
uint8_t y3=0;
uint8_t change=0;
uint8_t LED_ACTIVE=0;
int pom_int=0;
uint8_t direction=0;
uint8_t oldx=0;
uint8_t id2_flag=0;
uint8_t ID_duplicate=0;
int pp=0;
int k=0;
int n=0;
uint8_t rf=0;
uint8_t rf2=0;
uint8_t rf3=0;




// user function: is element present in int array?
int contain(int c, int p[], int size){
    int i;
    for (i = 0; i < size; i++){
        if(p[i]==c)return 1;
    }
    return 0;
}

// user function: is element present in uint8 array?
int contain2(uint8_t c, int p[], int size){
    int i;
    for (i = 0; i < size; i++) {
        if(p[i]==c){
            return 1;}
    }
    return 0;
}

// user function: is element present in uint8 array?
int contain3(uint8_t c, uint8_t p[], uint8_t size){
    int i;
    for (i = 0; i < size; i++) {
        if(p[i]==c){
            return 1;}
    }
    return 0;
}

// user function: fill uint8 array with zeros
void do_zeros2(uint8_t p[], int size){
    int i;
    for (i = 0; i < size; i++) {
        p[i]=0;
    }
}

// user function: fill int array with zeros
void do_zeros(int p[], int size){
    int i;
    for (i = 0; i < size; i++) {
        p[i]=0;
    }
}



// user function: set and record the message payload
void set_msg(uint8_t p1,uint8_t p2,uint8_t p3,uint8_t p4,uint8_t p5,uint8_t p6,uint8_t p7,uint8_t p8,uint8_t p9){
    int i;
    sending[0]=p1;
    sending[1]=p2;
    sending[2]=p3;
    sending[3]=p4;
    sending[4]=p5;
    sending[5]=p6;
    sending[6]=p7;
    sending[7]=p8;
    sending[8]=p9;

    for (i = 0; i < 9; i++) {
        message.data[i]=sending[i];

    }
    message.crc = message_crc(&message);
}


/*
  _____ ______ _______ _    _ _____
 / ____|  ____|__   __| |  | |  __ \
| (___ | |__     | |  | |  | | |__) |
 \___ \|  __|    | |  | |  | |  ___/
 ____) | |____   | |  | |__| | |
|_____/|______|  |_|   \____/|_|

 */

// Kilobot initialization before main loop
void setup() {

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    message.type = NORMAL;

    int temp = rand_hard();
    srand(((temp<<8)|rand_hard()));

    do_zeros(control_ID,255);
    MY_ID=(rand()%253)+1;
    //MY_ID=(kilo_uid%253)+1; usable only in ARGoS

    set_msg(MY_ID,0,0,0,0,0,0,0,phase);
    last_changed=kilo_ticks;
}



/*
 __  __          _____ _   _    _      ____   ____  _____
|  \/  |   /\   |_   _| \ | |  | |    / __ \ / __ \|  __ \
| \  / |  /  \    | | |  \| |  | |   | |  | | |  | | |__) |
| |\/| | / /\ \   | | | . ` |  | |   | |  | | |  | |  ___/
| |  | |/ ____ \ _| |_| |\  |  | |___| |__| | |__| | |
|_|  |_/_/    \_\_____|_| \_|  |______\____/ \____/|_|

*/


// MAIN LOOP
void loop() {


    //SHOWING ID - for visual kilobot malfunction check (could be omitted)
    if(kilo_ticks < 300){
        if(MY_ID<(kilo_ticks-last_changed))set_color(RGB(0,3,0));

    }


    if(new_message==1){


        //Unique seed to break symetry
        if (k<35)
        {
            n=n+received[0]+distance;
            k++;
            pp=n/k;
        }

        //Move into next phase if another robot already moved
        if(received[8]>phase){
            phase=received[8];
            last_changed=kilo_ticks;
            last_changed2=kilo_ticks;
            if(phase==9){
                step=1;
                //step=received[3];
                set_color(RGB(0,0,0));

            }
        }


        /*
         _____ _______       _____ _______    _____ _____    __
        / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  /_ |
       | (___    | |  /  \  | |__) | | |    | (___ | |__) |  | | __ _
        \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /   | |/ _` |
        ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   | | (_| |
       |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\  |_|\__,_|

       */



        //START Phase 0
        if(phase==0){



            //Generating additional ID(s)
            if(kilo_ticks > 200 && kilo_ticks < 300){
                if (id2_flag==0)
                {
                    srand(pp);

                    MY_ID2=(rand()%253)+1;

                    id2_flag=1;
                    set_msg(MY_ID,0,MY_ID2,0,0,0,0,0,phase);

                }


            }
            //Duplicity check

            if(received[3]==MY_ID&&kilo_ticks < 800){

                BlackList[blcounter]=MY_ID;
                blcounter++;

                srand(pp);

                MY_ID=(rand()%253)+1;

                while(contain2(MY_ID,BlackList,blcounter)){
                    MY_ID=(rand()%253)+1;
                }
                set_color(RGB(1,0,0));
                set_msg(MY_ID,0,MY_ID2,ID_duplicate,0,0,0,0,phase);
            }



            //Additional duplicity check (hearing 2 same IDs)



            if(kilo_ticks > 300 && kilo_ticks < 800){


                if(control_ID[received[0]]==0){
                    control_ID[received[0]]=received[2];
                }

                if(control_ID[received[0]]!=received[2]){
                    ID_duplicate=received[0];
                    set_msg(MY_ID,0,MY_ID2,ID_duplicate,0,0,0,0,phase);

                }


                if(contain2(received[0],BlackList,blcounter)==0&&received[0]!=MY_ID)
                {
                    BlackList[blcounter]=received[0];
                    blcounter++;

                }

                if(contain2(received[1],BlackList,blcounter)==0)
                {
                    BlackList[blcounter]=received[1];
                    blcounter++;

                }

                if(distance>=33){

                    // record the min. distance of incoming message
                    if(distance<min_distance)
                        min_distance=distance;


                }


            }

            /*
            ______ _   _ _____     _____ _____    __
           |  ____| \ | |  __ \   / ____|  __ \  /_ |
           | |__  |  \| | |  | | | (___ | |__) |  | | __ _
           |  __| | . ` | |  | |  \___ \|  _  /   | |/ _` |
           | |____| |\  | |__| |  ____) | | \ \   | | (_| |
           |______|_| \_|_____/  |_____/|_|  \_\  |_|\__,_|

           */




            /*
               _____ _______       _____ _______    _____ _____    __ _
              / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  /_ | |
             | (___    | |  /  \  | |__) | | |    | (___ | |__) |  | | |__
              \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /   | | '_ \
              ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   | | |_) |
             |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\  |_|_.__/

                   */


            //Neighborhood construction
            if(kilo_ticks > 800 && kilo_ticks < 1600) {

                if(min_distance>110)min_distance=110;

                if(contain2(received[0],neighbor_ID,neighbor_count)==0&&distance<(1.5*min_distance+10)&&distance>=33)
                {
                    neighbor_ID[neighbor_count]=received[0];
                    neighbor_DIST[neighbor_count]=distance;
                    neighbor_count++;

                }

            }



            if(kilo_ticks< 1700 && kilo_ticks > 800){


                if(neighbor_count==8)set_color(RGB(0,3,0)); // 8 GREEN
                if(neighbor_count==5)set_color(RGB(0,0,3)); // 5 BLUE
                if(neighbor_count==3)set_color(RGB(3,0,0)); // 3 RED
                if(neighbor_count>8)set_color(RGB(0,0,0)); // >8 black
                if(neighbor_count==7)set_color(RGB(3,3,0)); // 7 YELLOW
                if(neighbor_count==6)set_color(RGB(0,3,3)); // 6 CYAN
                if(neighbor_count==4)set_color(RGB(3,0,3)); // 4 MAGENTA
                if(neighbor_count==1)set_color(RGB(2,2,2)); // 1 dim WHITE
                if(neighbor_count==2)set_color(RGB(3,3,3)); // 2 WHITE
                if(neighbor_count==0)set_color(RGB(1,1,1)); // 0 dim dim WHITE

            }




            //Neighborhood construction - improving procedure
            if(kilo_ticks > 1000 && kilo_ticks < 1600){

                if(flag==0){


                    set_msg(MY_ID,neighbor_ID[g],0,0,0,0,0,66,phase);
                    flag=1;
                    last_changed=kilo_ticks;
                    set_color(RGB(0,0,0));

                }




                if(contain2(received[0],neighbor_ID,neighbor_count)==0&&received[1]==MY_ID&&received[7]==66)
                {
                    neighbor_ID[neighbor_count]=received[0];
                    neighbor_DIST[neighbor_count]=distance;
                    neighbor_count++;

                }


                if(kilo_ticks>(last_changed+30)){
                    g=g+1;
                    g=(g % neighbor_count);

                    set_msg(MY_ID,neighbor_ID[g],0,0,0,0,0,66,phase);
                    last_changed=kilo_ticks;
                }


                MY_STATE_value_min=100;
                MY_STATE_value_max=0;
            }


            /*
		  ______ _   _ _____     _____ _____    __ _
		 |  ____| \ | |  __ \   / ____|  __ \  /_ | |
		 | |__  |  \| | |  | | | (___ | |__) |  | | |__
		 |  __| | . ` | |  | |  \___ \|  _  /   | | '_ \
		 | |____| |\  | |__| |  ____) | | \ \   | | |_) |
		 |______|_| \_|_____/  |_____/|_|  \_\  |_|_.__/


            */


            /*
		   _____ _______       _____ _______    _____ _____    __
		  / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  /_ |
		 | (___    | |  /  \  | |__) | | |    | (___ | |__) |  | | ___
		  \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /   | |/ __|
		  ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   | | (__
		 |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\  |_|\___|

            */

            //Relative position detection
            if(kilo_ticks > 1650 && kilo_ticks < 1800 ){

                set_msg(MY_ID,neighbor_count,0,0,0,0,0,0,phase);
                if(contain2(received[0],neighbor_ID,neighbor_count)==1&&received[7]==0){
                    if(received[1]<MY_STATE_value_min)
                    {
                        MY_STATE_value_min=received[1];

                    };


                    if(received[1]>MY_STATE_value_max)MY_STATE_value_max=received[1];
                }



            }// assign mz relative position corner / border
            if(kilo_ticks > 1850){
                if(MY_STATE_value_min<=neighbor_count&&MY_STATE_value_max>neighbor_count)MY_STATE=1;
                if(neighbor_count<MY_STATE_value_min&&MY_STATE_value_max>neighbor_count)MY_STATE=2;



                if(MY_STATE==2){
                    set_msg(MY_ID,MY_ID2,0,0,0,0,0,0,phase);
                    set_color(RGB(3,0,0));


                }
                if(MY_STATE!=2){
                    set_msg(0,0,0,0,0,0,0,0,phase);

                }

            }

            if(kilo_ticks > 1950){
                phase=1;
                last_changed=kilo_ticks;
            }



        } //END Phase 0


        /*
          ______ _   _ _____     _____ _____    __
         |  ____| \ | |  __ \   / ____|  __ \  /_ |
         | |__  |  \| | |  | | | (___ | |__) |  | | ___
         |  __| | . ` | |  | |  \___ \|  _  /   | |/ __|
         | |____| |\  | |__| |  ____) | | \ \   | | (__
         |______|_| \_|_____/  |_____/|_|  \_\  |_|\___|

               */




        /*
    _____ _______       _____ _______    _____ _____    ___
   / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  |__ \
  | (___    | |  /  \  | |__) | | |    | (___ | |__) |    ) |__ _
   \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /    / // _` |
   ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   / /| (_| |
  |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\ |____\__,_|

        */

        //Phase 1 START - assigning axis origin {1,1}
        if(phase==1){

            //NOT corners
            pom=received[0];
            if(MY_STATE!=2&&pom>0){


                set_color(RGB(0,0,0));
                if(sending[0]==0 ||pom<sending[0]){
                    set_msg(pom,received[1],0,0,0,0,0,0,phase);

                }

            }
            //Corners
            if(MY_STATE==2&&pom>0){


                if(received[0]<MY_ID||(received[0]==MY_ID&&received[1]<MY_ID2)){
                    set_color(RGB(0,0,0));
                    active=0;

                    set_msg(received[0],received[1],0,0,0,0,0,0,phase);

                }


            }
            //Axis origin
            if(kilo_ticks>(last_changed+200)&&MY_STATE==2){
                if(active==1){
                    set_color(RGB(2,2,2));
                    im_origin=1;
                    x=1;
                    y=1;
                    f=1;
                    change=1;
                }

            }
            //Next phase setup
            if(change==1){
                change=0;
                phase=2;
                last_changed=kilo_ticks;
                pom=0;
                set_msg(MY_ID,x,y,0,0,0,0,0,phase);

            }

        }
        //Phase 1 END - assigning axis origin {1,1}



        //Phase 2 START  - assigning {1,2},{2,2},{2,1}
        if(phase==2){


            if(MY_STATE==0){

                if(received[1]==1 && received[2]==1 && x==0 && contain(received[0],neighbor_ID,neighbor_count)){
                    set_color(RGB(0,3,0));
                    x=2;
                    y=2;
                }

                set_msg(MY_ID,x,y,0,0,0,0,0,phase);

            }


            if(im_origin==1){

                if(received[1]==2 && received[2]==2 && pom==0 && contain(received[0],neighbor_ID,neighbor_count)){
                    set_color(RGB(0,3,3));
                    pom=1;

                    if(received[0]==neighbor_ID[0]){
                        set_msg(MY_ID,x,y,neighbor_ID[1],0,0,0,0,phase);
                    }

                    else set_msg(MY_ID,x,y,neighbor_ID[0],0,0,0,0,phase);


                }



            }



            if(kilo_ticks>(last_changed+150)){

                if(MY_STATE==1){

                    if(received[1]==1 && received[2]==1 &&received[3]>0 && x==0 && contain(received[0],neighbor_ID,neighbor_count)){

                        if(received[3]==MY_ID){
                            x=1;
                            y=2;
                            set_color(RGB(3,1,0));


                        }

                        if(received[3]!=MY_ID){
                            x=2;
                            y=1;
                            f=2;
                            set_color(RGB(0,1,3));
                        }


                    }


                }
            }
            //Next phase setup
            if(kilo_ticks>(last_changed+400)){
                set_msg(MY_ID,x,y,0,0,0,0,0,phase);
            }
            if(kilo_ticks>(last_changed+500)){
                phase=3;
                pom=0;
                last_changed=kilo_ticks;


            }

        }
        //Phase 2 END - assigning {1,2},{2,2},{2,1}

        /*
      ______ _   _ _____     _____ _____    ___
     |  ____| \ | |  __ \   / ____|  __ \  |__ \
     | |__  |  \| | |  | | | (___ | |__) |    ) |__ _
     |  __| | . ` | |  | |  \___ \|  _  /    / // _` |
     | |____| |\  | |__| |  ____) | | \ \   / /| (_| |
     |______|_| \_|_____/  |_____/|_|  \_\ |____\__,_|

        */





        /*
       _____ _______       _____ _______    _____ _____    ___  _
      / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  |__ \| |
     | (___    | |  /  \  | |__) | | |    | (___ | |__) |    ) | |__
      \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /    / /| '_ \
      ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   / /_| |_) |
     |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\ |____|_.__/

           */

        //Phase 3 START - assigning {1,2},{2,2},{2,1}

        if(phase==3){
            flag=0;

            if(im_origin || MY_STATE==1)set_msg(MY_ID,x,y,f,0,0,0,0,phase);
            if(MY_STATE==1 && (y!=2&&x!=1)){


                if(contain(received[0],neighbor_ID,neighbor_count)){

                    if(f==0&&received[3]>0){
                        f=received[3]+1;
                        set_color(RGB(3,3,2));
                    }
                }
            }


            if(MY_STATE==1 && y==2){


                if(contain(received[0],neighbor_ID,neighbor_count)&&received[1]!=2){

                    if(f==0&&received[3]>2){
                        f=received[3]+1;
                        set_color(RGB(3,0,0));
                    }
                }
            }
            if(im_origin ||MY_STATE==1)set_msg(MY_ID,x,y,f,0,0,0,0,phase);

            //Next phase setup
            if(im_origin&&contain(received[0],neighbor_ID,neighbor_count)&&received[3]>2){
                maxf=received[3];
                phase=4;
                last_changed=kilo_ticks;
                set_msg(MY_ID,x,y,f,0,0,0,0,phase);


            }

        } // Phase 3 END - assigning {1,2},{2,2},{2,1}


        // Phase 4 START - CORNERS ordering init.


        if(phase==4){

            if(flag==0){
                last_changed=kilo_ticks;
                set_msg(MY_ID,x,y,f,0,0,0,0,phase);
                flag=1;
            }


            if(im_origin==0&&MY_STATE==2&&contain(received[0],neighbor_ID,neighbor_count)&&received[3]>=2){
                if(cornerf1==0)cornerf1=received[3];
                if(cornerf1!=received[3])cornerf2=received[3];

                if(cornerf1!=0&&cornerf2!=0){
                    if(cornerf2<cornerf1)cornerf3=cornerf2;
                    else cornerf3=cornerf1;
                    f=cornerf3+1;
                }
            }

            if(kilo_ticks>(last_changed+250)){
                set_msg(MY_ID,0,0,0,0,0,0,MY_STATE,phase);
                set_color(RGB(0,0,0));
                cornerf1=0;
                cornerf2=0;
                cornerf3=0;

                if(MY_STATE==2&&im_origin==0){cornerf1=f;
                }
            }
            if(kilo_ticks>(last_changed+400)){
                phase=5;
                last_changed=kilo_ticks;

            }


        }
        // Phase 4 END - CORNERS ordering init.

        // Phase 5 START - CORNERS ordering
        if(phase==5){

            if(kilo_ticks>last_changed2+40){
                last_changed2=kilo_ticks;
                step=step+1;
                if(f==step)set_color(RGB(1,2,3));
            }


            if(MY_STATE==2&&im_origin==0){

                set_msg(MY_ID,f,0,0,0,0,0,MY_STATE,phase);
                if(received[2]!=received[3]&&received[3]!=received[4]&&received[2]!=received[4]&&received[2]!=0&&received[3]!=0&&received[4]!=0)
                {

                    if(f==received[2]){
                        fcounter=1;
                        set_color(RGB(3,0,0));maxX=f;
                    }


                    if(f==received[3]){
                        fcounter=2;
                        set_color(RGB(0,3,0));
                    }



                    if(f==received[4]){
                        fcounter=3;
                    }

                }


            }
            else{

                if(received[4]>0&&received[2]>0&&received[3]>0&&received[2]<received[3]&&received[3]<received[4]){

                    rf=received[2];
                    rf2=received[3];
                    rf3=received[4];

                }
                else{


                    if(received[1]>0&&rf==0){
                        rf=received[1];


                    }


                    if(received[1]>0&&received[1]!=rf&&rf2==0){
                        rf2=received[1];


                    }

                    if(received[1]>0&&received[1]!=rf&&received[1]!=rf2&&rf3==0){
                        rf3=received[1];


                    }


                    if(received[2]>0&&rf==0){
                        rf=received[2];

                    }

                    if(received[2]>0&&received[2]!=rf&&rf2==0){
                        rf2=received[2];


                        if(rf2<rf){
                            rf2=rf;
                            rf=received[2];
                        }
                    }

                    if(received[3]>0&&received[3]!=rf&&rf2==0){
                        rf2=received[3];

                        if(rf2<rf){
                            rf2=rf;
                            rf=received[3];
                        }
                    }

                    if(received[4]>0&&received[4]!=rf&&rf2==0){
                        rf2=received[4];

                        if(rf2<rf){
                            rf2=rf;
                            rf=received[4];
                        }
                    }

                    if(received[2]>0&&received[2]!=rf&&received[2]!=rf2&&rf3==0){
                        rf3=received[2];

                        if(rf3<rf){
                            rf3=rf2;
                            rf2=rf;
                            rf=received[2];
                        }
                        if(rf3>rf&&rf3<rf2){
                            rf3=rf2;
                            rf2=received[2];
                        }
                    }

                    if(received[3]>0&&received[3]!=rf&&received[3]!=rf2&&rf3==0){
                        rf3=received[3];

                        if(rf3<rf){
                            rf3=rf2;
                            rf2=rf;
                            rf=received[3];
                        }
                        if(rf3>rf&&rf3<rf2){
                            rf3=rf2;
                            rf2=received[3];
                        }

                    }

                    if(received[4]>0&&received[4]!=rf&&received[4]!=rf2&&rf3==0){
                        rf3=received[4];

                        if(rf3<rf){
                            rf3=rf2;
                            rf2=rf;
                            rf=received[4];
                        }
                        if(rf3>rf&&rf3<rf2){
                            rf3=rf2;
                            rf2=received[4];
                        }

                    }


                }

                set_msg(MY_ID,0,rf,rf2,rf3,0,0,MY_STATE,phase);



            }





            //Next phase setup
            if(fcounter==1){
                phase=6;
                last_changed=kilo_ticks;

            }

        }
        // Phase 5 END - CORNERS ordering





        /*
          ______ _   _ _____     _____ _____    ___  _
         |  ____| \ | |  __ \   / ____|  __ \  |__ \| |
         | |__  |  \| | |  | | | (___ | |__) |    ) | |__
         |  __| | . ` | |  | |  \___ \|  _  /    / /| '_ \
         | |____| |\  | |__| |  ____) | | \ \   / /_| |_) |
         |______|_| \_|_____/  |_____/|_|  \_\ |____|_.__/

                */



        /*
    _____ _______       _____ _______    _____ _____    ___
   / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  |__ \
  | (___    | |  /  \  | |__) | | |    | (___ | |__) |    ) |___
   \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /    / // __|
   ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   / /| (__
  |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\ |____\___|

        */

        // Phase 6 START - Assigning coordinates to CORNERS and BORDERS
        if(phase==6){
            set_msg(MY_ID,x,y,0,0,maxX,maxY,0,phase);

            if(received[5]>0){maxX=received[5];}
            if(received[6]>0){maxY=received[6];}

            if(maxX>0){set_color(RGB(1,3,1));}


            if(fcounter==2&&maxX>0){maxY=(f-maxX+2);}


            if(maxX>0&&maxY>0){
                set_color(RGB(3,0,0));

                //Assign coordinates to CORNERS
                if(fcounter==1&&im_origin==0){x=maxX;y=1;}
                if(fcounter==2&&im_origin==0){x=maxX;y=maxY;}
                if(fcounter==3&&MY_STATE==2){x=1;y=maxY;}


                //Assign coordinates to BORDERS
                if(MY_STATE==1&&x==0&&y==0){
                    if(f<maxX){
                        x=f;
                        y=1;
                    }
                    if(f>=maxX&&f<(maxY+maxX-2)){
                        x=maxX;
                        y=f-maxX+2;

                    }

                    if(f>=(maxY+maxX-2)&&f<maxX+maxX+maxY-4){
                        x=maxX+maxX+maxY-4-f+1;
                        y=maxY;
                    }

                    if(f>=maxX+maxX+maxY-4){
                        x=1;

                        y=maxY-(f-(maxX+maxX+maxY-4))-1;

                    }
                }

            }



            //Next phase setup
            if(kilo_ticks>(last_changed+400)){

                phase=7;
                last_changed=kilo_ticks;
                last_changed2=kilo_ticks;
                set_msg(MY_ID,x,y,0,0,maxX,maxY,0,phase);
                step=0;
                set_color(RGB(0,0,0));
            }

        } // Phase 6 END - Assigning coordinates to CORNERS and BORDERS


        // Phase 7 & 8 START - Assigning coordinates to robots INSIDE the formation & visual check

        if(phase==7||phase==8){
            //step=0;
            if(im_origin){y=1;}

            // Computing the coordinates of robots INSIDE the formation
            if(MY_STATE==0){
                if(contain(received[0],neighbor_ID,neighbor_count)&&received[1]!=0){
                    pom=received[1];
                    if(x1==0){x1=pom;}
                    if(pom!=x1&&x2==0){x2=pom;}
                    if(pom!=x1&&pom!=x2&&x3==0){x3=pom;}
                    set_msg(MY_ID,x,y,0,0,maxX,maxY,0,phase);

                    pom=received[2];
                    if(yy1==0){yy1=pom;}
                    if(pom!=yy1&&y2==0){y2=pom;}
                    if(pom!=yy1&&pom!=y2&&y3==0){y3=pom;}
                    set_msg(MY_ID,x,y,0,0,maxX,maxY,0,phase);
                }


                if(x1!=0&&x2!=0&&x3!=0&&x==0){

                    x=(x1+x2+x3)/3;
                }

                if(yy1!=0&&y2!=0&&y3!=0&&y==0){

                    y=(yy1+y2+y3)/3;
                }

            }

            // Visual check X
            if(phase==7){


                if(x==0){set_color(RGB(3,0,0));}
                else if(x%5==1){set_color(RGB(0,3,0));}
                else if(x%5==2){set_color(RGB(0,0,3));}
                else if(x%5==3){set_color(RGB(3,0,3));}
                else if(x%5==4){set_color(RGB(3,3,0));}
                else if(x%5==0){set_color(RGB(3,3,3));}


                // If center hit
                if(kilo_ticks>(last_changed+200)){
                    if(x==maxX/2||x==(maxX+1)/2){
                        if(y==maxY/2||y==(maxY+1)/2){
                            phase=8;
                            last_changed=kilo_ticks;
                            last_changed2=kilo_ticks;
                            set_msg(MY_ID,x,y,0,0,maxX,maxY,0,phase);
                        }

                    }

                }
            }

        }



        // Visual check Y
        if(phase==8){


            if(y==0){set_color(RGB(3,0,0));}
            else if(y%5==1){set_color(RGB(0,3,0));}
            else if(y%5==2){set_color(RGB(0,0,3));}
            else if(y%5==3){set_color(RGB(3,0,3));}
            else if(y%5==4){set_color(RGB(3,3,0));}
            else if(y%5==0){set_color(RGB(3,3,3));}


            if(kilo_ticks>(last_changed+200)){
                step=0;
                phase=9;
                last_changed=kilo_ticks;
                last_changed2=kilo_ticks;
                set_msg(MY_ID,x,y,step,0,maxX,maxY,0,phase);


            }




        }// Phase 7 & 8 END - Assigning coordinates to robots INSIDE the formation & visual check


        /*
  ______ _   _ _____     _____ _____    ___
 |  ____| \ | |  __ \   / ____|  __ \  |__ \
 | |__  |  \| | |  | | | (___ | |__) |    ) |___
 |  __| | . ` | |  | |  \___ \|  _  /    / // __|
 | |____| |\  | |__| |  ____) | | \ \   / /| (__
 |______|_| \_|_____/  |_____/|_|  \_\ |____\___|

       */


        /*
   _____ _______       _____ _______    _____ _____    ____
  / ____|__   __|/\   |  __ \__   __|  / ____|  __ \  |___ \
 | (___    | |  /  \  | |__) | | |    | (___ | |__) |   __) | __ _
  \___ \   | | / /\ \ |  _  /  | |     \___ \|  _  /   |__ < / _` |
  ____) |  | |/ ____ \| | \ \  | |     ____) | | \ \   ___) | (_| |
 |_____/   |_/_/    \_\_|  \_\ |_|    |_____/|_|  \_\ |____/ \__,_|

       */


        // Phase 9 START - DISPLAY MESSAGE
        if(phase==9){

            set_msg(MY_ID,x,y,step,0,maxX,maxY,0,phase);

            // SYNC check
            if(received[3]==(step+1)||(step==(letters-1)&&received[1]==0)){

                step=received[3];
                last_changed2=kilo_ticks;
                set_color(RGB(0,0,0));
                LED_ACTIVE=0;
            }
            if(p+kilo_ticks>(last_changed2+130)){
                set_color(RGB(0,0,0));
                p=0;
                last_changed2=kilo_ticks;
                step++;
                step=(step%letters);
                LED_ACTIVE=0;

            }

            // LED activation
            if((step%letters)==1){
                for(int f=0;f<2*lengths[0];f=f+2){

                    if(x==p1[f]&&y==p1[f+1])LED_ACTIVE=1;



                }

            }

            // LED activation
            if((step%letters)==2){
                for(int f=0;f<2*lengths[1];f=f+2){

                    if(x==p2[f]&&y==p2[f+1])LED_ACTIVE=1;



                }
            }

            // LED activation
            if((step%letters)==3){
                for(int f=0;f<2*lengths[2];f=f+2){

                    if(x==p3[f]&&y==p3[f+1])LED_ACTIVE=1;



                }
            }

            // LED activation
            if((step%letters)==4){
                for(int f=0;f<2*lengths[3];f=f+2){

                    if(x==p4[f]&&y==p4[f+1])LED_ACTIVE=1;



                }
            }

            // LED activation
            if((step%letters)==0){
                for(int f=0;f<2*lengths[letters-1];f=f+2){

                    if(x==p4[f]&&y==p4[f+1])LED_ACTIVE=1;


                }
            }
            if(LED_ACTIVE)set_color(RGB(0,3,0));


        } // // Phase 9 END - DISPLAY MESSAGE


        /*
		  ______ _   _ _____     _____ _____    ____
		 |  ____| \ | |  __ \   / ____|  __ \  |___ \
		 | |__  |  \| | |  | | | (___ | |__) |   __) | __ _
		 |  __| | . ` | |  | |  \___ \|  _  /   |__ < / _` |
		 | |____| |\  | |__| |  ____) | | \ \   ___) | (_| |
		 |______|_| \_|_____/  |_____/|_|  \_\ |____/ \__,_|

               */






        new_message=0;
    } // END MESSAGE







    //new_message=0;

};

// message handling
void message_rx(message_t *m, distance_measurement_t *d)
{
#ifdef ARGOS
    if(new_message==0||(rand()%1000)<500){
#else
        if(new_message==0){
#endif
        distance=estimate_distance(d);
        for (k = 0; k < 9; k++){
            received[k]=m->data[k];
        }


        new_message = 1;
    }

}

message_t *message_tx()
{
    return &message;
}

void message_tx_success() {
    sent_message = 1;
}


// Kilobot main
int main() {
    // initialize hardware
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    // start program
    kilo_start(setup, loop);

    return 0;
}
