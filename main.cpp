#include "mbed.h"
#include "quadroschrauber.h"
#include "rtos.h"

QuadroSchrauber schrauber;


        Timer timer2;



void blink(void const *n) {
            if(schrauber.remote.active == false && schrauber.timer.read_ms() > 2000)
                schrauber.remote.active = true;
                
            int delta = timer2.read_us();
            //if(delta == 0)
            //    return;
            float dtime = ((float)delta) / 1000000.0f;
            timer2.reset();
            timer2.start();
            
            schrauber.Update(0.01f);
}

int main() {
    //if (schrauber.wd.WatchdogCausedReset())
    //{
        //schrauber.pc.printf("Watchdog caused reset.\r\n");
    //}
    
    //schrauber.wd.Configure(1.0);
    
    
    if (schrauber.ok)
    {
        schrauber.timer.reset();
        schrauber.timer.start();
    
        timer2.reset();
        timer2.start();
        
        
        RtosTimer led_1_timer(blink, osTimerPeriodic, (void *)0);
    
        led_1_timer.start(10);
        Thread::wait(osWaitForever);
        
        while (schrauber.ok)
        {
            wait(0.1);
        }
    }

    
    
    
    schrauber.Shutdown();
    
    while(true)
    {
        //schrauber.wd.Service();
        wait(0.01f);
    };
}
