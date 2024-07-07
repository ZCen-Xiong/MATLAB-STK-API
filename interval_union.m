% 将多段求并集
function result = interval_union(data)
    % 按照第一列排序
    data = sortrows(data, 1);

    % 初始化结果矩阵
    result = data(1, :);

    % 遍历剩下的数据
    for i = 2:size(data, 1)
        % 检查当前段的开始是否在结果矩阵的最后一段的范围内
        if data(i, 1) <= result(end, 2)
            % 如果在，则更新结果矩阵最后一段的结束
            result(end, 2) = max(data(i, 2), result(end, 2));
        else
            % 如果不在，则将当前段加入到结果矩阵中
            result = [result; data(i, :)];
        end
    end
end
