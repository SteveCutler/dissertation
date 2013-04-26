classdef KTMMatlabRead
    properties
        hasDepth = 0
        depthWidth = 0
        depthHeight = 0
        
        hasRGB = 0
        RGBWidth = 0
        RGBHeight = 0
        EOF = 1
    end
    properties(SetAccess = private)
        dataStart = 0
        depthFrameDataSize = 0
        RGBFrameDataSize = 0
        totalFrameDataSize = 0
        frames = 0
        currentFrame = 0
    end
    methods
        function obj = openFile(obj, filePath)
            obj.hasDepth = 0;
            obj.depthWidth = 0;
            obj.depthHeight = 0;

            obj.hasRGB = 0;
            obj.RGBWidth = 0;
            obj.RGBHeight = 0;
            %'C:\Users\ganterd\Desktop\Dissertation\KTMKinectRecorder\output\saved.bin'
            file = fopen(filePath);
            fileInfo = fread(file,5,'*char').';
            obj.hasDepth = strcmp(fileInfo, 'DEPTH');
            if(obj.hasDepth)
                fileInfo = char(fread(file,8)).';
                if(strcmp(fileInfo,'80X60   '))
                    obj.depthWidth = 80;
                    obj.depthHeight = 60;
                elseif(strcmp(fileInfo,'320X240 '))
                    obj.depthWidth = 320;
                    obj.depthHeight = 240;
                elseif(strcmp(fileInfo,'640X480 '))
                    obj.depthWidth = 640;
                    obj.depthHeight = 480;
                end
                fprintf('Depth stream: %i x %i\n', obj.depthWidth, obj.depthHeight)
            else
                fseek(file,-5,'cof');
                fprintf('Has no Depth Stream\n')
            end


            fileInfo = fread(file,5,'*char').';
            obj.hasRGB = strcmp(fileInfo, 'RGB  ');
            if(obj.hasRGB)
                fileInfo = char(fread(file,8)).';
                if(strcmp(fileInfo,'640X480 '))
                    obj.RGBWidth = 640;
                    obj.RGBHeight = 480;
                elseif(strcmp(fileInfo,'1280X960'))
                    obj.RGBWidth = 1290;
                    obj.RGBHeight = 960;
                end
                fprintf('RGB stream: %i x %i\n', obj.RGBWidth, obj.RGBHeight)
            else
                fseek(file,-5,'cof');
                fprintf('Has no RGB stream.\n')
            end
            
            obj.dataStart = ftell(file);
            obj.depthFrameDataSize = (4 + 5 + (obj.depthWidth * obj.depthHeight * 2)) * obj.hasDepth;
            obj.RGBFrameDataSize = (4 + 5 + (obj.RGBWidth * obj.RGBHeight * 4)) * obj.hasRGB;
            obj.totalFrameDataSize = obj.depthFrameDataSize + obj.RGBFrameDataSize;;
            fseek(file,0,'eof');
            dataEnd = ftell(file);
            dataSize = dataEnd - obj.dataStart;
            obj.frames = dataSize / obj.totalFrameDataSize;
            fprintf('Total frames: %i\n', obj.frames);
        end
    end
end