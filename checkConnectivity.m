function isConnected = checkConnectivity(adjMatrix)
    % 构建图对象
    G = graph(adjMatrix, 'upper');

    % 计算连通分量的数量
    numComponents = conncomp(G);

    % 如果连通分量的数量为1，则表示图是连通的
    isConnected = (numComponents == 1);
end