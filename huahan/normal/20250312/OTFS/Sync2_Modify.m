function [peakIndices, sync_op] = Sync2_Modify(noisySignal,norm_matched_filter,frame_size) %#codegen
% peak_width = 128;
% shift_flag = 0;
sync_op = conv(norm_matched_filter,noisySignal);
sync_op_abs = abs(sync_op);

% stf_detection_threshold = 0.7* max(abs(sync_op));  

numberToKeep = floor(length(sync_op_abs) / frame_size) * frame_size;
% reshapedSignal = reshape(sync_op_abs(1:numberToKeep), frame_size, []);
% frame_number = size(reshapedSignal, 2);

% Initialize arrays to store peak values and indices
[~,peakIndices] = findpeaks(sync_op_abs(1:numberToKeep),'MinPeakHeight', 0.75 * max(sync_op_abs),'MinPeakDistance',frame_size);

% %%==================================================================
% %%first adjustment
% for frame = 1:2
%     % Extract the current segment
%     segment = reshapedSignal(:, frame);
% 
%     % Find the peak in the current segment
%     [~, locs] = max(segment);
% 
%     peakIndices(frame) = locs+(frame-1)*frame_size;
% end
% 
% if(peakIndices(2) - peakIndices(1)) <= 128
%     sync_op_abs = sync_op_abs(peak_width:end);
%     shift_flag = 1;
% end
% 
% reshapedSignal = reshape(sync_op_abs(1:numberToKeep), frame_size, []);
% frame_number = size(reshapedSignal, 2);
% %%==================================================================
% 
% % Loop through each segment to find peaks
% for frame = 1:frame_number
%     % Extract the current segment
%     segment = reshapedSignal(:, frame);
% 
%     % Find the peak in the current segment
%     [peak_value, locs] = max(segment);
%     if  shift_flag == 1
%         peakIndices(frame) = locs+(frame-1)*frame_size+peak_width-1;
%     else
%         peakIndices(frame) = locs+(frame-1)*frame_size;
%     end
% 
%     % if frame == 1 && peak_value < stf_detection_threshold
%     %     fake_peak = 1;
%     % end
end
