import math

def calcular_perimetro_area():
    radio = float(input("Introduce el radio del círculo: "))
    perimetro = 2 * math.pi * radio
    area = math.pi * radio ** 2

    print(f"El perímetro del círculo es: {perimetro}")
    print(f"El área del círculo es: {area}")

if __name__ == "__main__":
    calcular_perimetro_area()