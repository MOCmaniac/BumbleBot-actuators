// Serial input variables
const byte numChars = 10;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  // Démarrez la communication série avec une vitesse de 115200 bauds
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Waiting for commands");
}

void loop() {
  // Si des données sont disponibles sur le port série
  recvWithStartEndMarkers();
  //recChatGPT();
  
  if(newData){
    handleCommand(receivedChars);
    newData = false;
  }
}

// Code from Robin2 on arduino's website : Serial Input Basics
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0) {  // Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    Serial.printf("Received character: %c\n", rc);

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        Serial.printf("Received message: %s\n", receivedChars);
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void recChatGPT() {
  if (Serial.available() > 0) {
    // Lire la commande depuis le port série
    String commande = Serial.readStringUntil('\n');

    // Trouver les indices du début et de la fin de la commande
    int debut = commande.indexOf('<');
    int fin = commande.indexOf('>');
    // Si les indices sont valides et dans le bon ordre
    if (debut != -1 && fin != -1 && debut < fin) {
      // Extraire la partie de la commande entre les marqueurs < et >
      String commandeExtraite = commande.substring(debut + 1, fin);

      // Afficher la commande extraite
      Serial.println(commandeExtraite);
      newData = true;
    } else {
      // Marqueurs manquants ou dans le mauvais ordre
      Serial.println("Commande mal formée");
    }
  }
}

void handleCommand(char *string) {
  char command;
  int value;
  sscanf(string, "%c-%d", &command, &value);

  switch (command) {
    case 'F':  // Fork pot
      Serial.println("Fork pot");
      break;
    case 'f':  // Fork plant
      Serial.println("Fork plant");
      break;
    case 'G':  // Plants gripper
      Serial.println("Plants gripper");
      break;
    case 'T':  // Forks translation
      Serial.println("Plants translation");
      break;
    default:
      Serial.println("Command not recognized !");
      break;
  }
  Serial.printf("Command received : %s\n", string);
}