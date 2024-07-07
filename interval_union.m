% ������󲢼�
function result = interval_union(data)
    % ���յ�һ������
    data = sortrows(data, 1);

    % ��ʼ���������
    result = data(1, :);

    % ����ʣ�µ�����
    for i = 2:size(data, 1)
        % ��鵱ǰ�εĿ�ʼ�Ƿ��ڽ����������һ�εķ�Χ��
        if data(i, 1) <= result(end, 2)
            % ����ڣ�����½���������һ�εĽ���
            result(end, 2) = max(data(i, 2), result(end, 2));
        else
            % ������ڣ��򽫵�ǰ�μ��뵽���������
            result = [result; data(i, :)];
        end
    end
end
