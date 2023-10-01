from collections import deque

# Definição da classe para Grafo com representação em Matriz de Adjacência
class GrafoMatrizAdjacencia:
    def __init__(self, vertices):
        self.vertices = vertices
        # Inicialização da matriz de adjacência com 0's
        self.grafo = [[0] * vertices for _ in range(vertices)]

    def adicionar_aresta(self, u, v):
        # Adiciona uma aresta entre os vértices u e v
        self.grafo[u][v] = 1
        self.grafo[v][u] = 1

    def imprimir_matriz_adjacencia(self):
        # Imprime a matriz de adjacência
        for linha in self.grafo:
            print(linha)

# Definição da classe para Grafo com representação em Lista de Adjacência
class GrafoListaAdjacencia:
    def __init__(self, vertices):
        self.vertices = vertices
        # Inicialização do dicionário que representa a lista de adjacência
        self.grafo = {}

    def adicionar_aresta(self, u, v):
        # Adiciona uma aresta entre os vértices u e v na lista de adjacência
        if u not in self.grafo:
            self.grafo[u] = []
        if v not in self.grafo:
            self.grafo[v] = []

        self.grafo[u].append(v)
        self.grafo[v].append(u)

    def imprimir_lista_adjacencia(self):
        # Imprime a lista de adjacência
        for vertice, vizinhos in self.grafo.items():
            print(f'{vertice}: {", ".join(map(str, vizinhos))}')

# Função para busca em largura (BFS)
def bfs(grafo, s, t):
    fila = deque()
    visitados = set()
    predecessores = {}

    fila.append(s)
    visitados.add(s)

    while fila:
        vertice = fila.popleft()
        if vertice == t:
            caminho = []
            while vertice is not None:
                caminho.append(vertice)
                vertice = predecessores.get(vertice)
            return caminho[::-1]

        for vizinho in grafo.get(vertice, []):
            if vizinho not in visitados:
                fila.append(vizinho)
                visitados.add(vizinho)
                predecessores[vizinho] = vertice

    return None

# Função para busca em profundidade (DFS)
def dfs(grafo, s):
    visitados = set()
    pilha = []

    pilha.append(s)
    visitados.add(s)

    while pilha:
        vertice = pilha.pop()
        print(vertice, end=' ')

        for vizinho in grafo.get(vertice, []):
            if vizinho not in visitados:
                pilha.append(vizinho)
                visitados.add(vizinho)

if __name__ == "__main__":
    num_vertices = 6
    grafo_matriz = GrafoMatrizAdjacencia(num_vertices)
    grafo_matriz.adicionar_aresta(0, 1)
    grafo_matriz.adicionar_aresta(0, 2)
    grafo_matriz.adicionar_aresta(1, 3)
    grafo_matriz.adicionar_aresta(2, 4)
    grafo_matriz.adicionar_aresta(3, 5)

    print("Matriz de Adjacência:")
    grafo_matriz.imprimir_matriz_adjacencia()

    grafo_lista = GrafoListaAdjacencia(num_vertices)
    grafo_lista.adicionar_aresta(0, 1)
    grafo_lista.adicionar_aresta(0, 2)
    grafo_lista.adicionar_aresta(1, 3)
    grafo_lista.adicionar_aresta(2, 4)
    grafo_lista.adicionar_aresta(3, 5)

    print("\nLista de Adjacência:")
    grafo_lista.imprimir_lista_adjacencia()

    s, t = 0, 5
    caminho_bfs = bfs(grafo_lista.grafo, s, t)
    if caminho_bfs:
        print(f"\nCaminho BFS de {s} para {t}: {caminho_bfs}")
    else:
        print(f"\nNão há caminho entre {s} e {t}")

    print("\nCaminho DFS:")
    dfs(grafo_lista.grafo, s)
