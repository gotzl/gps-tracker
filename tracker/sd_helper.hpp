/*
 * sd_helper.cpp
 *
 *  Created on: May 14, 2019
 *  Most of the functions are taken (or adapted) from the SD_Test included in ESP32-arduino platform
 */


#include "FS.h"
#include "SD.h"


int16_t createSessionDir(fs::FS &fs) {
	int16_t session = -1;
    uint16_t n_files = 0;
    char session_dir[12];

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
            	int16_t sess=atoi(file.name()+8);
            	// why does this not work?
//            	sscanf(file.name(), "/session%3hi", &sess);
            	if (sess>=0 && sess>session) {
            		session = sess;
            		latest_session = file;
            	}
            }
        }
        file = root.openNextFile();
    }
    root.close();

    // test if the latest session folder has some content
    if (session>=0) {
    	file = latest_session.openNextFile();
		while(file) {
			file = file.openNextFile();
			n_files += 1;
		}

		// take empty latest session folder
		if (n_files==0) {
			Serial.print( F("Taking existing session dir: ") );
			Serial.println( latest_session.name() );
			latest_session.close();
			return session;
		} else {
			Serial.print( F("Latest session dir: ") );
			Serial.print( latest_session.name() );
			Serial.print( F(" files: " ) );
			Serial.println( n_files );
			latest_session.close();
		}
    }

    // create new session folder
    session += 1;

    snprintf(session_dir, sizeof session_dir, "/session%03i", session);
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
        Serial.println( F("Failed to open file for writing") );
        return;
    }
    if(file.write(buf, size)){
        Serial.println( F("File written") );
    } else {
        Serial.println( F("Write failed") );
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const uint8_t *buf, size_t size){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println( F("Failed to open file for appending") );
        return;
    }
    if(file.write(buf, size)){
        Serial.println( F("Message appended") );
    } else {
        Serial.println( F("Append failed") );
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



bool setup_sd() {
	uint8_t cardType;
	uint64_t cardSize;

	// the SD library seems to need to allocate
	// big chunks of data when max_files=5, we don't have
	// this much connected data any more ... -> limit max_files=2
	if (!SD.begin(SS, SPI, 4000000, "/sd", 2)) {
		Serial.println("Card Mount Failed");
		return false;
	}

	cardType = SD.cardType();
	if (cardType == CARD_NONE) {
		Serial.println("No SD card attached");
		return false;
	}

	Serial.print("SD Card Type: ");
	if (cardType == CARD_MMC) {
		Serial.println("MMC");
	} else if (cardType == CARD_SD) {
		Serial.println("SDSC");
	} else if (cardType == CARD_SDHC) {
		Serial.println("SDHC");
	} else {
		Serial.println("UNKNOWN");
	}

	cardSize = SD.cardSize() / (1024 * 1024);
	Serial.printf("SD Card Size: %lluMB\n", cardSize);

	listDir(SD, "/", 0);
	return true;
}
