/*
 * sd_helper.cpp
 *
 *  Created on: May 14, 2019
 *  Most of the functions are taken (or adapted) from the SD_Test included in ESP32-arduino platform
 */


#include "FS.h"
#include "SD.h"

uint16_t createSessionDir(fs::FS &fs) {
	int16_t session = -1;

    File root = fs.open("/");
    if(!root){
        Serial.println("Failed to open root directory");
        return -1;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return -1;
    }

    File file = root.openNextFile();
    File latest_session;
    while(file){
        if(file.isDirectory()){
            if (strncmp("/session", file.name(), 8) == 0) {
            	int16_t sess=-1;
            	sscanf(file.name(),"/session%03i", &sess);
            	if (sess>=0 && sess>session) {
            		session = sess;
            		latest_session = file;
            	}
            }
        }
        file = root.openNextFile();
    }

    // test if the latest session folder has some content
    uint16_t n_files=0;
    if (session>=0) {
    	File file = latest_session.openNextFile();
		while(file) {
			file = file.openNextFile();
			n_files += 1;
		}

		// take empty latest session folder
		if (n_files==0) {
			Serial.print( F("Taking existing session dir: ") );
			Serial.println( latest_session.name() );
			return session;
		}
    }

    // create new session folder
    session += 1;

    char session_dir[11];
    sprintf(session_dir, "/session%03i",session);
    Serial.print( F("Creating session dir: ") );
    Serial.print( session_dir );
    if(fs.mkdir(session_dir)){
        Serial.println( F(" ... Success!") );
    } else {
        Serial.println( F(" ... Failed!") );
    }

    return session;
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const uint8_t *buf, size_t size){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.write(buf, size)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}


