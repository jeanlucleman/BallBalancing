#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <vector>


#include "FS.h"

// #ifdef esp8266
// #include <fs.h>
// #elif defined ESP_32
// #include <SPIFFS.h>
// #endif

using namespace std;
typedef struct
  {
    String key;
    String value;
    String description;
  }  keyValue;

// vector<String> split(String my_str, String separator); 

// This class allows to create, edit, use settings in a structure (key, value, description). 
// These settings are save in a file through SPIFFS. The settings can be created and modified by the funtions
// setValue(). The value and description can be accessed by the property getValue() and getDecsiption().
// When a settings is created or modified, it is automarically saved in the settings.txt file.
class Settings {       
  private:
    // Dynamic array containind items in the structure 'keyValue'
    vector <keyValue> _listKeyValues;
    // function to read the keyValue items and store them in the dynamic array _listKeyValues
    vector <keyValue> _readItems();

    // Flag to check if the Settings object has been properly initialized. The full initialization cannot be done in the class constructor.
    // The reason is that the instantiation is made before the main setup is started and this creates a program crash...
    bool _ini;

  public:          
    // Function to complete initialisation of the Settings object
    String _fileName; 
    void ini();
    String test;
    // Constructor of Settings class
    Settings(String fileName); 
    
    File getFile(const char* mode);
    String getKey(unsigned int i);
    // Function to get the value corresponding to the given key
    String getValue(String key);
    // Function to get the value corresponding to the given key, and obtain a default value in case the setting is not existing
    String getValue(String key, String defaultValue);
    // Function to get the description corresponding to the given key
    String getDescription(String key);
    // Defines the value or record identified by its key. Allows also to add a description of the record
    // - String key: the unique identifier of the record
    // - String value: the value of this record
    // - String decsription: a short description of the record
    bool setValue(String key, String value, String description);
    bool setValue(String key, String value);
    bool setValue(String key, String value, String description, bool autosave);
    bool setValue(String key, String value, bool autosave);
    // function to save the settings in a file (settings.txt)
    void save();
    // Returns true if the record collection contains a record with the key identifier
    bool contains(String key);
    // Returns the number of records in the collection
    int count();
    // ALlows to erase one record identified by its key. Returns false if the record is not existing.
    bool deleteMe(String key);
    // Returns on the console the list of records
    String list();
    // Erase the flash area (in fact, just delete the values saved, but keep the keys)
    void erase();
    

  };





#endif 