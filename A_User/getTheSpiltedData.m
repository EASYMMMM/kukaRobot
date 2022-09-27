function [data_train_input,data_train_output] = getTheSpiltedData(all_data, index_set, v_state)
%���� getTheBeginEndIndex() �Ľ���õ��ķָ�������
        data_train_input = []; data_train_output = [];
        for j = 1:size(index_set, 1)
            data_train_input_temp = all_data(index_set(j, 1): index_set(j, 2) - 1, :); % ѵ��
            data_train_output_temp = all_data(index_set(j, 1) + 1: index_set(j, 2), :); % label
            data_train_input = [data_train_input; data_train_input_temp];
            data_train_output = [data_train_output; data_train_output_temp];
        end
        data_train_input = [data_train_input, v_state * ones(size(data_train_input, 1), 1)];
end

