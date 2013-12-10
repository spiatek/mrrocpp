#include <cmath>
#include <cstring>
#include <ostream>

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                           definicje skladowych klasy Ft_v_tr
// ******************************************************************************************

Ft_v_tr::Ft_v_tr()
{
}

std::ostream& operator<<(std::ostream & strumien, Ft_v_tr & m)
{
	// operator wypisania
	// przedstawia macierz w przyjaznej dla czlowieka formie

	for (int j = 0; j < 6; j++) {
		for (int i = 0; i < 6; i++) {
			strumien << m.matrix_m[j][i] << "\t\t";
		}
		strumien << std::endl;
	}

	return strumien;
}

// ******************************************************************************************
//                                           definicje skladowych klasy Xi_f
// ******************************************************************************************

Xi_f::Xi_f() :
		Ft_v_tr()
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++) {
			if (i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}
}

Xi_f::Xi_f(const Homog_matrix & p) :
		Ft_v_tr()
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	base_frame = p;

	set_from_frame(base_frame);
}

// konstruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Xi_f::Xi_f(const Xi_f &wzor)
{
	base_frame = wzor.base_frame;

	set_from_frame(wzor.base_frame);
}

Xi_f Xi_f::operator*(const Xi_f & m) const
{
	// mnozenie macierzy

	Xi_f zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xi_f Xi_f::operator*(const Xi_star & m) const
{
	// mnozenie macierzy

	Xi_f zwracana(base_frame * m.base_frame);

	return zwracana;
}

Ft_vector Xi_f::operator*(const Ft_vector & w) const
{
	Ft_vector zwracany;

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int j = 0; j < 6; j++)
		for (int i = 0; i < 6; i++)
			zwracany[j] += matrix_m[j][i] * w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
	return zwracany;
}

Xi_f & Xi_f::operator=(const Xi_f & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	memcpy(matrix_m, wzor.matrix_m, sizeof(matrix_m));

	base_frame = wzor.base_frame;

	return *this;
}

Xi_f Xi_f::operator!() const
{
	// przeksztalcenie odwrotne

	Xi_f zwracana;

	zwracana.base_frame = !base_frame;

	zwracana.set_from_frame(zwracana.base_frame);

	return zwracana;
}

void Xi_f::set_from_frame(const Homog_matrix & p)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.70, 5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			matrix_m[i][j] = p(i, j);
			matrix_m[i + 3][j + 3] = p(i, j);
			matrix_m[j][i + 3] = 0;
		}

	double Porg[3][3];

	Porg[0][0] = 0;
	Porg[0][1] = -p(2, 3);
	Porg[0][2] = p(1, 3);

	Porg[1][0] = p(2, 3);
	Porg[1][1] = 0;
	Porg[1][2] = -p(0, 3);

	Porg[2][0] = -p(1, 3);
	Porg[2][1] = p(0, 3);
	Porg[2][2] = 0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_m[j + 3][i] = 0;
			for (int a = 0; a < 3; a++) {
				matrix_m[j + 3][i] += Porg[j][a] * p(a, i);
			}
		}
	}
}

// ******************************************************************************************
//                                           definicje skladowych klasy Xi_v
// ******************************************************************************************

Xi_v::Xi_v() :
		Ft_v_tr()
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}
	}
}

Xi_v::Xi_v(const Homog_matrix & p) :
		Ft_v_tr()
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	base_frame = p;

	set_from_frame(base_frame);
}

// konstruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Xi_v::Xi_v(const Xi_v &wzor)
{
	base_frame = wzor.base_frame;

	set_from_frame(wzor.base_frame);
}

Xi_v Xi_v::operator*(const Xi_v & m) const
{
	// mnozenie macierzy

	Xi_v zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xi_v Xi_v::operator*(const Xi_star & m) const
{
	// mnozenie macierzy

	Xi_v zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xyz_Angle_Axis_vector Xi_v::operator*(const Xyz_Angle_Axis_vector & w) const
{
	Xyz_Angle_Axis_vector zwracany;

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int j = 0; j < 6; j++)
		for (int i = 0; i < 6; i++)
			zwracany[j] += matrix_m[j][i] * w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
	return zwracany;
}

Xi_v & Xi_v::operator=(const Xi_v & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	memcpy(matrix_m, wzor.matrix_m, sizeof(matrix_m));

	base_frame = wzor.base_frame;

	return *this;
}

Xi_v Xi_v::operator!() const
{
	// przeksztalcenie odwrotne
	Xi_v zwracana;

	zwracana.base_frame = !base_frame;

	zwracana.set_from_frame(zwracana.base_frame);

	return zwracana;
}

void Xi_v::set_from_frame(const Homog_matrix & p)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.70, 5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_m[i][j] = p(i, j);
			matrix_m[i + 3][j + 3] = p(i, j);
			matrix_m[j + 3][i] = 0;
		}
	}

	double Porg[3][3];

	Porg[0][0] = 0;
	Porg[0][1] = -p(2, 3);
	Porg[0][2] = p(1, 3);

	Porg[1][0] = p(2, 3);
	Porg[1][1] = 0;
	Porg[1][2] = -p(0, 3);

	Porg[2][0] = -p(1, 3);
	Porg[2][1] = p(0, 3);
	Porg[2][2] = 0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_m[j][i + 3] = 0;
			for (int a = 0; a < 3; a++) {
				matrix_m[j][i + 3] += Porg[j][a] * p(a, i);
			}
		}
	}
}

// ******************************************************************************************
//                                           definicje skladowych klasy Xi_star
// ******************************************************************************************

Xi_star::Xi_star() :
		Ft_v_tr()
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}
	}
}

Xi_star::Xi_star(const Homog_matrix & p) :
		Ft_v_tr()
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	base_frame = p;

	set_from_frame(base_frame);
}

// konstruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Xi_star::Xi_star(const Xi_star &wzor)
{
	base_frame = wzor.base_frame;

	set_from_frame(wzor.base_frame);
}

Xi_star Xi_star::operator*(const Xi_star & m) const
{
	// mnozenie macierzy

	Xi_star zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xi_star Xi_star::operator*(const Xi_v & m) const
{
	// mnozenie macierzy

	Xi_star zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xi_star Xi_star::operator*(const Xi_f & m) const
{
	// mnozenie macierzy

	Xi_star zwracana(base_frame * m.base_frame);

	return zwracana;
}

Xyz_Angle_Axis_vector Xi_star::operator*(const Xyz_Angle_Axis_vector & w) const
{
	Xyz_Angle_Axis_vector zwracany;

	// i - i-ta kolumna
	// j - j-ty wiersz
	for (int j = 0; j < 6; j++)
		for (int i = 0; i < 6; i++)
			zwracany[j] += matrix_m[j][i] * w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
	return zwracany;
}

Ft_vector Xi_star::operator*(const Ft_vector & w) const
{
	Ft_vector zwracany;

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int j = 0; j < 6; j++)
		for (int i = 0; i < 6; i++)
			zwracany[j] += matrix_m[j][i] * w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
	return zwracany;
}

Xi_star & Xi_star::operator=(const Xi_star & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if (this == &wzor)
		return *this;

	memcpy(matrix_m, wzor.matrix_m, sizeof(matrix_m));

	base_frame = wzor.base_frame;

	return *this;
}

Xi_star Xi_star::operator!() const
{
	// przeksztalcenie odwrotne
	Xi_star zwracana;

	zwracana.base_frame = !base_frame;

	zwracana.set_from_frame(zwracana.base_frame);

	return zwracana;
}

void Xi_star::set_from_frame(const Homog_matrix & p)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.70, 5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_m[i][j] = p(i, j);
			matrix_m[i + 3][j + 3] = p(i, j);
			matrix_m[j + 3][i] = 0;
		}
	}

	double Porg[3][3];

	Porg[0][0] = 0;
	Porg[0][1] = 0;
	Porg[0][2] = 0;

	Porg[1][0] = 0;
	Porg[1][1] = 0;
	Porg[1][2] = 0;

	Porg[2][0] = 0;
	Porg[2][1] = 0;
	Porg[2][2] = 0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix_m[j][i + 3] = 0;
			for (int a = 0; a < 3; a++) {
				matrix_m[j][i + 3] += Porg[j][a] * p(a, i);
			}
		}
	}
}

} // namespace lib
} // namespace mrrocpp

