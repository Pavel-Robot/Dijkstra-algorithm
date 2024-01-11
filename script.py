# -*- coding: cp1251 -*-

import sys
import os
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# �������� ������ ����
# � ������������ ���������� ����������� ����
# ����������� ��������� ��������
class Graph():
    def __init__(self, vertices):
        self.V = vertices #������� �����
        self.graph = [[0 for column in range(vertices)] #������� ���������
                      for row in range(vertices)]
    
    # ����� ��� ����������� �������
    def printSolution(self, dist):
        s = {}
        print("������� \t���������� �� ���������")
        for node in range(self.V):
            s[node] = dist[node]
            print(node, "\t\t", dist[node])
        return s
  
    # ������� ��� ���������� ������������ ���������� �� ������� �������
    # ����: dist ������ ��������� � sptSet ������ ���������� ������
    def minDistance(self, dist, sptSet):
  
        # ���������������� ����������� ���������� ��� ���������� ����
        min = sys.maxsize
  
        # ���� ����, �� �������� ������� ����� ���������
        # �� ��������� �������
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
        
        #������ ������ ����� �������
        return min_index
  
    # �������, ����������� �������� ��������
    # �������� ������ ����������� ���� ��� ��������������� �����
    # src - ��� ������� �� ������� �� ��� �� �����
    def dijkstra(self, src):
  
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
  
        for cout in range(self.V):
            
            # �������� ������� ������������ ���������� ��
            # ������ ��� �� ������������ ������.
            # x ������ ����� src � ������ ��������
            x = self.minDistance(dist, sptSet)
            

            # ���� �� ������� ������� ������������ ����������, ��
            # ������� �� ����������
            sptSet[x] = True

            
  
            # �������� �������� ���������� �������� ������
            # ��������� �������, ������ ���� �������
            # ���������� ������ ������ ���������� �
            # ������� �� � ������ ���������� ������
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]
                    #print("�� ������� " + str(x) + " ���� ����� � " + str(y) + " �������� " + str(self.graph[x][y]) + " ����� �������: " + str(dist[y]) )
  

        return self.printSolution(dist)
  
  
if __name__ == "__main__":

    #�������� �����
    path = os.path.split(__file__)

    #��������� ������� ��������� ��������� �� �����, ����������� ���� �� ������
    if True:
        #���� ����� ����
        with open(path[0] + "\\" + "data.txt", "r") as graphTxt:
            graphTxtInput = graphTxt.read().split("\n")
        vertexSize = len(graphTxtInput)
        g = Graph(vertexSize)
        for i in range(vertexSize):
            buf = list(graphTxtInput[i].split(","))
            buf = [int(buf[j]) for j in range(len(buf))]
            g.graph[i] = buf[::1]

    else:
        #���� �������

        #������� ���� g � 9 ��������� 0,1,2,...,8
        g = Graph(5)
        g.graph = [[0, 0, 3, 6, 5],
                   [0, 0, 9, 6, 2],
                   [3, 9, 0, 3, 9],
                   [6, 6, 3, 0, 1],
                   [5, 2, 9, 1, 0]]
    
    #������� �� ����� ������� ����� �������� ���������� ���� � ��������� �������� �������� ��� �����
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
  