
/************************************************************************************
 * ESP has no physical clock but a counter of milliseconds elapsed since powered on. This number of milliseconds is given by the function millis().
 * This class provides a virtual clock computing time by : t = t0 + m - m0 where:
 * - t0 is the time when it is provided from outside to the ESP -> lastTimeUpdate, in seconds (number of seconds elapsed since Unix Epoch, 1/1/1070)
 * - m0 is the millis value at this time -> millisLastTimeUpdate, in milliseconds so to be divided by 1000
 * - m is the millis value when time is computed -> millis() function
 * There are 2 ways to get time from outside
 * - from a NTP server -> manage by this class -> Now deprecated!
 * - from outside this class (ex. from the central unit to which ESP is connected -> managed outside of this class by exchange of http request between UC and ESP).
 * The programme managing this has to call void Clock::refreshClockFromUC(unsigned long now) to provide the t0
 * The class object is a singleton and is created by Clock* myClock=Clock::getInstance(decalage , 1); 
 * Date and time can be retrieved as string by the functions getNow_YYYY_MM_DD_HH_MM_SS();
 *      getDateTime_YYYY_MM_DD_HH_MM_SS, getNow_YYYY_MM_DD(), getNow_HH_MM_SS(), getNow_HH_MM_SS_MMM();
 * 
  Jean-Luc LEMAN - Juin 2020
*************************************************************************************/

#ifndef CLOCK_H
  #define CLOCK_H
  #include <Arduino.h>
  const int EPOCH_1_1_2019 = 1546300800;
  class Clock
    {
      public:
        static Clock* getInstance(); // Class method as the only access to the Singleton Object
        // gmtOffset = time shift in hour between Greenwitch meridian and local
        // daylightOffset = time offset in summer time (day saving light)
        //static Clock* getInstance(int gmtOffset, int daylightOffset); 
        // gmtOffset = time shift in hour between Greenwitch meridian and local
        // daylightOffset = time offset in summer time (day saving light)
        // periodRefreshClockMillis: Period in milliseconds to call the NTP server and refresh the time on ESP (default = 86400000)
        //static Clock* getInstance(int gmtOffset, int daylightOffset, unsigned long periodRefreshClockMillis); // Class method as the only access to the Singleton Object
        //static const unsigned long defaultPeriodRefreshClockMillis = 86400000; // 24 heures
        // input now is the to in seconds from Unix Epoch. This method will save this value as well as the millis at this time.
        void setClockFromUC(unsigned long now);
        // This function returns the current date and time as 2020/06/09 18:28:02
        String getNow_YYYY_MM_DD_HH_MM_SS();
        // This function returns the date and time for a given timeMillis as 2020/06/09 18:28:02
        String getDateTime_YYYY_MM_DD_HH_MM_SS(unsigned long timeMillis);
        // This function returns the current date in the form 2020/06/09
        String getNow_YYYY_MM_DD();
        // This function returns the current time in the form 18:28:02
        String getNow_HH_MM_SS();
        // This function returns the current time in the form 18:28:02.123 (with the milliseconds)
        String getNow_HH_MM_SS_MMM();
        //void setDst(int dst);   
        void setGmtOffset(int gmtOffset);

        // This function starts the clock. It must be lauched after the Wifi is setup.
        // ntpServer= address of the NTP server (default = pool.ntp.org)
        bool begin(String ntpServer);
        // Time when the clock has been refreshed (by a call on the NTP server or from UC)
        //time_t lastTimeUpdate;
        unsigned long  lastTimeUpdate=0;
        time_t localDateTime();
      private:
        static Clock* pInstance; // Static variable holding the pointer to the only instance of this
        //Clock(int gmtOffset, int daylightOffset, unsigned long periodRefreshClockMillis);
        Clock();

        // This function returns the current date time by adding the last time the clock was updated and the millis elapsed since this time
        struct tm * getLocalTime(unsigned long timeMillis);
        unsigned long millisLastTimeUpdate = 0; // moment de la plus récente mise à jour de l'affichage de l'heure
        int _gmtOffset=0;
        //int _dayLightOffset = 0;
        String _ntpServer;
        //unsigned long _periodRefreshClockMillis;
    };
#endif