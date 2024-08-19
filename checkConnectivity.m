function isConnected = checkConnectivity(adjMatrix)
    % ����ͼ����
    G = graph(adjMatrix, 'upper');

    % ������ͨ����������
    numComponents = conncomp(G);

    % �����ͨ����������Ϊ1�����ʾͼ����ͨ��
    isConnected = (numComponents == 1);
end