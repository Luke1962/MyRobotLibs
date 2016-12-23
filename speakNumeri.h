//#include <iostream.h> 
//#include <stdlib.h> 
//#include <string.h> 

const int numGruppi = 3;
const char *gruppoName[] = { "mila", "milioni" };
const char *irregular[] = { "mille", "unmilione" };

const char *cifre19[] = { "zero", "uno", "due", "tre", "quattro", "cinque",
"sei", "sette", "otto", "nove", "dieci", "undici",
"dodici", "tredici", "quattordici", "quindici",
"sedici", "diciassette", "diciotto", "diciannove" };

const char *decine[] = { "venti", "trenta", "quaranta", "cinquanta",
"sessanta", "settanta", "ottanta", "novanta" };



bool Cifra(const char *ch);
int moveEfondo(char array[], int size);
void ConvLetter(const char cifreIn[], int size);

bool Cifra(const char *str)
{

	for (; *str != '\0'; str++)
	{
		if ((*str < 48) || (*str > 57))
			return false;
	}

	return true;


}
int moveEfondo(char array[], int size)
{
	int lungChars = strlen(array),
		sposta = (size - 1);

	if (lungChars < size)
	{
		for (int i = 0; i < lungChars; i++)
		{
			array[sposta] = array[((lungChars - 1) - i)];
			sposta--;
		}

		for (int j = 0; j < (size - lungChars); j++)
			array[j] = 48;
	}

	if (size < 1)
		return -1;

	else return 0;
}
void ConvLetter(const char cifreIn[], int size)
{


	void Centinaia(int cifre[], int group);
	void Decine(int cifre[], int group);
	void Gruppi(int cifre[], int group);


	int gCifre[numGruppi] = { 0 };




	for (int gruppo = (numGruppi - 1), copia = 0; gruppo >= 0; gruppo--, copia += 3)
	{


		gCifre[gruppo] = ((cifreIn[copia] - 48) * 100 +
			(cifreIn[copia + 1] - 48) * 10 +
			(cifreIn[copia + 2] - 48));
	}


	int zero = 0;


	for (int chkGroup = (numGruppi - 1); chkGroup >= 0; chkGroup--)
	{

		if (gCifre[chkGroup] == 0)
			zero++;

		else
		{
			Centinaia(gCifre, chkGroup);


			Decine(gCifre, chkGroup);


			Gruppi(gCifre, chkGroup);


		}

	}


	if (zero == numGruppi)
		cout << cifre19[0];


	cout << endl;

}
void Centinaia(int cifre[], int group)
{


	if ((cifre[group] / 100) > 1)
		cout << cifre19[(cifre[group] / 100)];   // "due"..."nove" 


	if ((cifre[group] / 100) > 0)
	{
		if (((cifre[group] % 100) / 10) == 8)
			cout << "cent";
		else cout << "cento";
	}

}
void Decine(int cifre[], int group)
{


	if (((cifre[group] % 100) < 20) && ((cifre[group] % 100) > 0))
	{

		if ((group == 0)
			|| ((group > 0) && (cifre[group] != 1)))
			cout << cifre19[(cifre[group] % 100)];

	}
	else
	{

		if (((cifre[group] % 100) > 20) &&
			(((cifre[group] % 10) == 1) || ((cifre[group] % 10) == 8)))
		{
			char temp[9] = "";

			cout << strncpy(temp,
				decine[(((cifre[group] % 100) / 10) - 2)],
				(strlen(decine[(((cifre[group] % 100) / 10) - 2)]) - 1));
		}

		else if ((cifre[group] % 100) >= 20)
			cout << decine[(((cifre[group] % 100) / 10) - 2)];

		if (((cifre[group] % 100) % 10) > 0)
			cout << cifre19[((cifre[group] % 100) % 10)];

	}

}
void Gruppi(int cifre[], int group)
{
	if (group > 0)
	{
		if (cifre[group] > 1)
			cout << gruppoName[group - 1];

		else cout << irregular[group - 1];
	}

}


void number2text(char input[[numGruppi * 3], char txt[20])
{

	//char input[numGruppi * 3];

	//cout << "Digita un intero (max " << (numGruppi * 3) << " cifre): ";
	//cin >> input;

	if (Cifra(input) == true)
	{

		moveEfondo(input, numGruppi * 3);

		cout << endl;


		ConvLetter(input, numGruppi * 3);

	}

	else
	{
		dbg("Error")
		/*cout << endl
			<< "ERRORE: Input non valido!" << endl
			<< "Riavviare il programma e digitare soltanto numeri."
			<< endl << endl;*/
	}
	//system("PAUSE");
	//return 0;
}
