% function make_simscape_video(model_name, video_name)
opt = struct('PlaybackSpeedRatio',4,'FrameSize',[810 540]);
write_simscape_video(model_name, video_name, opt);

fprintf('Please wait until Simscape finishes writing all the individual videos and then press any key...\n');
pause;
% concatenate tile videos into a single video using ffmpeg
% (source: https://stackoverflow.com/questions/11552565/vertically-or-horizontally-stack-mosaic-several-videos-using-ffmpeg)
system(sprintf('ffmpeg -y -i %s_1.mp4 -i %s_3.mp4 -i %s_2.mp4 -i %s_4.mp4 -filter_complex "[0:v][1:v][2:v][3:v]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[v]" -map "[v]" %s.mp4', video_name, video_name, video_name, video_name, video_name));
fprintf('Please wait until ffmpeg finishes combining the videos and then press any key...\n');
pause
add_watermark_video(video_name)
add_watermark_video(sprintf('%s_4',video_name), [.95 .95]);
% system(sprintf('ffmpeg -y -i %s.mp4 -i watermark.png -filter_complex "overlay=x=(main_w-overlay_w)/2:y=(main_h-overlay_h)/2" %s_watermarked.mp4&', video_name, video_name));
% fprintf('Done\n');
% end

function make_simscape_video(model_name, video_name, varargin)
    narginchk(2,3);
    
    if nargin > 2
        opt = varargin{1};
    else
        opt = struct();
    end
    if ~isfield(opt,'Tile')
        opt.Tile = 1:4;
    end
    if ~isfield(opt,'VideoFormat')
        opt.VideoFormat = 'mpeg-4';
    end
    if ~isfield(opt,'PlaybackSpeedRatio')
        opt.PlaybackSpeedRatio = 4;
    end  
    if ~isfield(opt,'FrameRate')
        opt.FrameRate = 60;
    end  
    if ~isfield(opt,'FrameSize')
        opt.FrameSize = 'auto';
    end  
    
    for tile = opt.Tile
        smwritevideo(model_name,sprintf('%s_%d',video_name, tile),...
                     'Tile',tile,'VideoFormat',opt.VideoFormat,...
                     'PlaybackSpeedRatio',opt.PlaybackSpeedRatio,...
                     'FrameRate', opt.FrameRate, 'FrameSize', opt.FrameSize);
    end
end