#include <NTPClient.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>


// Insert your network credentials
#define WIFI_SSID "Free_wifi"
#define WIFI_PASSWORD "sariawan"
// Insert Firebase project API Key
#define API_KEY "AIzaSyB4Co6y38mdB2A-0t_BjMBZzmnDw4QNeI4"

// Insert RTDB URL
#define DATABASE_URL "https://data-sensor-51f5e-default-rtdb.asia-southeast1.firebasedatabase.app" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

#define ONE_WIRE_BUS 4 // Pin untuk sensor suhu (misalnya, menggunakan pin D2)
#define SOIL_MOISTURE_SENSOR A0 // Pin untuk sensor kelembaban tanah (misalnya, menggunakan pin A0)
const int relayPin = 5; // D1


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

FirebaseJson json;


#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);


// Number of inputs to the fuzzy inference system
const int fis_gcI = 2;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 12;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

bool fuzzyProcessFinished = true;

// Nilai kalibrasi untuk sensor
int AirValue = 700;   // Nilai sensor saat tanah kering sepenuhnya
int WaterValue = 465; // Nilai sensor saat terendam air sepenuhnya

// Variabel untuk menyimpan jumlah pengukuran dan total kelembaban dan suhu per 1 menit
int totalMeasurements = 0;
float totalSoilMoisture = 0;
float totalTemperature = 0;

// Setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

  /* Assign the api key (required) */
    config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
    config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

  // Start reading temperature from DS18B20 sensor
    sensors.begin();

    pinMode(SOIL_MOISTURE_SENSOR, INPUT); // Mengatur pin sensor kelembaban tanah sebagai input
    pinMode(relayPin, OUTPUT); // Mengatur pin output untuk valve

    // Set NTP client to get time
    timeClient.begin();
}

// Loop routine runs over and over again forever:
void loop() {
    unsigned long currentTime = millis();

    if (Firebase.ready() && signupOK) {
        // Reset total measurements and total soil moisture and temperature
        totalMeasurements = 0;
        totalSoilMoisture = 0;
        totalTemperature = 0;
        
        // Request temperature sensor to read temperature
        sensors.requestTemperatures();

        // Loop to collect data per 15 minutes
        while (totalMeasurements < 300) { // Collect data for 15 minutes (900 seconds)
            // Read temperature from the sensor
            float temperatureCelsius = sensors.getTempCByIndex(0);
            
            // Read analog value from soil moisture sensor
            int soilMoistureReading = analogRead(SOIL_MOISTURE_SENSOR);
            
            // Calculate soil moisture value
            int soilMoisture = map(soilMoistureReading, AirValue, WaterValue, 0, 100);
            soilMoisture = constrain(soilMoisture, 0, 100);

            // Add soil moisture and temperature to total
            totalSoilMoisture += soilMoisture;
            totalTemperature += temperatureCelsius;
            totalMeasurements++;

            delay(1000); // Delay 1 second between each measurement
        }

        // Calculate average soil moisture and temperature
        float avgSoilMoisture = totalSoilMoisture / totalMeasurements;
        float avgTemperature = totalTemperature / totalMeasurements;

        // Evaluate fuzzy logic
        g_fisInput[0] = avgSoilMoisture; // Set soil moisture as input
        g_fisInput[1] = avgTemperature; // Set temperature as input
        g_fisOutput[0] = relayPin;

        fis_evaluate();

        // Set output value: Valve
        int fuzzyOutputValue = g_fisOutput[0]; // Save fuzzy output value
        // digitalWrite(relayPin, fuzzyOutputValue); // Control the relay based on fuzzy output value

        // Print fuzzy output value to Serial Monitor
        Serial.print("Output Fuzzy: ");
        Serial.println(fuzzyOutputValue);

        // Convert fuzzyOutputValue (seconds) to milliseconds
        unsigned long relayDuration = fuzzyOutputValue * 1000; // Convert seconds to milliseconds

        // Activate relay if fuzzyOutputValue is not zero
        if (fuzzyOutputValue != 0) {
            digitalWrite(relayPin, LOW); // Turn on the relay
            sendSensorDataToFirebase(avgSoilMoisture, avgTemperature, fuzzyOutputValue);

            delay(relayDuration); // Wait for the specified duration
            digitalWrite(relayPin, HIGH); // Turn off the relay after the duration
        }else{
            digitalWrite(relayPin, HIGH);
            sendSensorDataToFirebase(avgSoilMoisture, avgTemperature, fuzzyOutputValue);

        }

    }

    // Update NTP time
    timeClient.update();
}


void sendSensorDataToFirebase(float soilMoisture, float temperature, int fuzzyOutputValue) {
    // Kirim data ke Firebase Realtime Database

        // Get current timestamp
        unsigned long currentTimestamp = getTime();

        // Path untuk menyimpan data dengan timestamp sebagai ID unik
        String path = "/logs/sensors/" + String(currentTimestamp);

        // Tambahkan data ke objek JSON
        FirebaseJson json;
        json.set("/soil_moisture", soilMoisture);
        json.set("/temperature", temperature);
        json.set("/fuzzy_value", fuzzyOutputValue);
        json.set("/timestamp", currentTimestamp);

        // Kirim data ke Firebase
        Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
    
}



//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) fmax(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trapmf, fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 4 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 4 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -10, 0, 50, 60 };
FIS_TYPE fis_gMFI0Coeff2[] = { 50, 60, 80 };
FIS_TYPE fis_gMFI0Coeff3[] = { 78, 90, 100, 110 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 10, 15, 18, 23 };
FIS_TYPE fis_gMFI1Coeff2[] = { 19, 23, 27 };
FIS_TYPE fis_gMFI1Coeff3[] = { 25, 30, 37 };
FIS_TYPE fis_gMFI1Coeff4[] = { 34, 37, 40, 42 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0, 240 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0, 480 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0, 720 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 0 };
int fis_gMFI1[] = { 0, 1, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1};

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1 };
int fis_gRI1[] = { 1, 2 };
int fis_gRI2[] = { 1, 3 };
int fis_gRI3[] = { 1, 4 };
int fis_gRI4[] = { 2, 1 };
int fis_gRI5[] = { 2, 2 };
int fis_gRI6[] = { 2, 3 };
int fis_gRI7[] = { 2, 4 };
int fis_gRI8[] = { 3, 1 };
int fis_gRI9[] = { 3, 2 };
int fis_gRI10[] = { 3, 3 };
int fis_gRI11[] = { 3, 4 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11 };

// Rule Outputs
int fis_gRO0[] = { 2 };
int fis_gRO1[] = { 3 };
int fis_gRO2[] = { 3 };
int fis_gRO3[] = { 4 };
int fis_gRO4[] = { 1 };
int fis_gRO5[] = { 1 };
int fis_gRO6[] = { 1 };
int fis_gRO7[] = { 2 };
int fis_gRO8[] = { 1 };
int fis_gRO9[] = { 1 };
int fis_gRO10[] = { 1 };
int fis_gRO11[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 15 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 100, 40 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
// None for Sugeno

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = 0;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            FIS_TYPE sWI = 0.0;
            for (j = 0; j < fis_gOMFCount[o]; ++j)
            {
                fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
                for (i = 0; i < fis_gcI; ++i)
                {
                    fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR; ++r)
            {
                index = fis_gRO[r][o] - 1;
                sWI += fuzzyFires[r] * fuzzyOutput[o][index];
            }

            g_fisOutput[o] = sWI / sW;
        }
    }
}