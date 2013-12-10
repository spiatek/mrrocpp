#ifndef __FT_V_TR_H
#define __FT_V_TR_H

#include <ostream>

namespace mrrocpp {
namespace lib {

class Xi_f;
class Xi_v;
class Xi_star;

// klasa reprezentujaca macierz transformacji odczytow sily do innego ukladu odniesienia

class Ft_v_tr
{
protected:
	double matrix_m[6][6]; // zmienna przechowujaca parametry macierzy

public:
	Homog_matrix base_frame; // bazowy trojscian z konstruktora
	Ft_v_tr(); // kostruktor domniemany
	// destruktur wirtualny
	virtual ~Ft_v_tr()
	{
	}

	virtual void set_from_frame(const Homog_matrix & p) = 0; // ustawia na podstawie trojscianu

	friend std::ostream& operator<<(std::ostream & strumien, Ft_v_tr &); // operator wypisania
};

class Xi_f : public Ft_v_tr
{
public:
	Xi_f(); // kostruktor domniemany
	Xi_f(const Homog_matrix &);
	Xi_f(const Xi_f &); // konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p); // ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Xi_f operator*(const Xi_f & m) const;
	Xi_f operator*(const Xi_star & m) const;

	Xi_f operator!() const;

	Xi_f & operator =(const Xi_f &); // operator przypisania
	Ft_vector operator*(const Ft_vector &) const; // mnozenie wektora
};

class Xi_v : public Ft_v_tr
{
public:
	Xi_v(); // kostruktor domniemany
	Xi_v(const Homog_matrix &);
	Xi_v(const Xi_v &); // konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p); // ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Xi_v operator*(const Xi_v & m) const;
	Xi_v operator*(const Xi_star & m) const;
	Xi_v operator!() const;

	Xi_v & operator =(const Xi_v &); // operator przypisania
	Xyz_Angle_Axis_vector operator*(const Xyz_Angle_Axis_vector &) const; // mnozenie wektora
};

class Xi_star : public Ft_v_tr
{
public:
	Xi_star(); // kostruktor domniemany
	Xi_star(const Homog_matrix &);
	Xi_star(const Xi_star &); // konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p); // ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Xi_star operator*(const Xi_star & m) const;
	Xi_star operator*(const Xi_v & m) const;
	Xi_star operator*(const Xi_f & m) const;
	Xi_star operator!() const;

	Xi_star & operator =(const Xi_star &); // operator przypisania
	Xyz_Angle_Axis_vector operator*(const Xyz_Angle_Axis_vector &) const; // mnozenie wektora
	Ft_vector operator*(const Ft_vector &) const; // mnozenie wektora
};

} // namespace lib
} // namespace mrrocpp

#endif
