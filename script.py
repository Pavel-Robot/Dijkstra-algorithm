# -*- coding: cp1251 -*-

import sys
import os
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Описание класса граф
# с функционалом нахождения кратчайшего пути
# посредством алгоритма Дейкстры
class Graph():
    def __init__(self, vertices):
        self.V = vertices #Вершины графа
        self.graph = [[0 for column in range(vertices)] #матрица смежности
                      for row in range(vertices)]
    
    # Метод для отображения решения
    def printSolution(self, dist):
        s = {}
        print("Вершина \tРасстояние от источника")
        for node in range(self.V):
            s[node] = dist[node]
            print(node, "\t\t", dist[node])
        return s
  
    # Функция для нахождения минимального расстояния до смежной вершины
    # Вход: dist список дистанций и sptSet список посещенных вершин
    def minDistance(self, dist, sptSet):
  
        # Инициализировать минимальное расстояние для следующего узла
        min = sys.maxsize
  
        # Ищем путь, по которому быстрее всего доберемся
        # до следующей вершины
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
        
        #Вернем индекс такой вершины
        return min_index
  
    # Функция, реализующая алгоритм Дейкстры
    # алгоритм поиска кратчайшего пути для представленного графа
    # src - это вершина от которой мы идём до конца
    def dijkstra(self, src):
  
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
  
        for cout in range(self.V):
            
            # Выбираем вершину минимального расстояния из
            # набора еще не обработанных вершин.
            # x всегда равен src в первой итерации
            x = self.minDistance(dist, sptSet)
            

            # Если мы выбрали вершину минимального расстояния, то
            # считаем ее посещенной
            sptSet[x] = True

            
  
            # Обновить значение расстояния соседних вершин
            # выбранной вершины, только если текущее
            # расстояние больше нового расстояния и
            # вершина не в списке посещенных вершин
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]
                    #print("От вершины " + str(x) + " могу пойти в " + str(y) + " потратив " + str(self.graph[x][y]) + " общие затраты: " + str(dist[y]) )
  

        return self.printSolution(dist)
  
  
if __name__ == "__main__":

    #Корневая папка
    path = os.path.split(__file__)

    #Заполняем матрицу смежности состоящую из нулей, расставляем веса по ребрам
    if True:
        #Если через файл
        with open(path[0] + "\\" + "data.txt", "r") as graphTxt:
            graphTxtInput = graphTxt.read().split("\n")
        vertexSize = len(graphTxtInput)
        g = Graph(vertexSize)
        for i in range(vertexSize):
            buf = list(graphTxtInput[i].split(","))
            buf = [int(buf[j]) for j in range(len(buf))]
            g.graph[i] = buf[::1]

    else:
        #Если вручную

        #Создаем граф g с 9 вершинами 0,1,2,...,8
        g = Graph(5)
        g.graph = [[0, 0, 3, 6, 5],
                   [0, 0, 9, 6, 2],
                   [3, 9, 0, 3, 9],
                   [6, 6, 3, 0, 1],
                   [5, 2, 9, 1, 0]]
    
    #Говорим от какой вершины будем находить кратчайшие пути и запускаем алгоритм Дейкстры для графа
    g.dijkstra(0)

    
    G1 = nx.Graph(np.matrix(g.graph))
    #pos = nx.shell_layout(G1)
    #pos = nx.spring_layout(G1)
    pos = nx.random_layout(G1)
    nx.draw_networkx(G1, pos, with_labels=True, node_size = 500, node_color='#ff704d', edge_color='g') #arrows=None arrowsize=0,
    label = nx.get_edge_attributes(G1,'weight')
    nx.draw_networkx_edge_labels(G1,pos, edge_labels=label)
    plt.show()

    print("End")
  