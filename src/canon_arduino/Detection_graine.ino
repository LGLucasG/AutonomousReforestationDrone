int photoResistorPin = A0; // Pin de la photodiode
int photoResistorValue = 0; 
int led = 13; // Led de test

void setup() {
  Serial.begin(9600); // Initialisation de la com pour visualisation sur ordinateur
  pinMode(led, OUTPUT);
}

void loop() {
  photoResistorValue = analogRead(photoResistorPin); // Read the value from the photodiode
  if (photoResistorValue < 300) { // seuil à définir quand les LEDs et photodiodes sont dans le système (autrement biais de la lumière ambiante)
    digitalWrite(led, HIGH); // La bille est détectée, on fait quelque chose (tourner le moteur par exemple)
  }
  else {
    digitalWrite(led, LOW); // La bille n'est plus détectée
  }
  Serial.println(photoResistorValue); 
  delay(100); // Pourra être supprimé lors de l'utilisation, ne sert qu'à visualiser les résultats sur le Serial Monitor
}
