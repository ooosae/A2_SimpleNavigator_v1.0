import numpy as np

def generate_weighted_adjacency_matrix(n, directed=False):
    # Генерация случайных весов для ребер
    weights = np.random.randint(1, 10, size=(n, n))
    
    # Задание главной диагонали нулями (граф не содержит петель)
    np.fill_diagonal(weights, 0)
    
    # Создание матрицы смежности
    if directed:
        return weights
    else:
        # Создание симметричной матрицы для неориентированного графа
        adjacency_matrix = np.triu(weights) + np.triu(weights, 1).T
        return adjacency_matrix

def save_matrix_to_file(matrix, filename):
    with open(filename, 'w') as file:
        file.write(f"{len(matrix)}\n")
        np.savetxt(file, matrix, fmt='%d')

def main():
    n = int(input("Введите размерность графа: "))
    directed = input("Граф будет ориентированным? (да/нет): ").lower() == 'да'
    filename = input("Введите имя файла для сохранения матрицы: ")

    adjacency_matrix = generate_weighted_adjacency_matrix(n, directed)
    save_matrix_to_file(adjacency_matrix, filename)

    print(f"Матрица смежности сохранена в файле {filename}")

if __name__ == "__main__":
    main()

