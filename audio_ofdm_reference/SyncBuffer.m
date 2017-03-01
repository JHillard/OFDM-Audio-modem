classdef SyncBuffer
    properties
        buffer
        numElem
        alignIndex
    end
    methods
        function obj = SyncBuffer()
            obj.buffer     = 0;
            obj.numElem    = 0;
            obj.alignIndex = 0;
        end
        function obj = setup(obj, val)
            obj.numElem = val;
            obj.buffer = zeros(2 * val, 1);
            obj = reset(obj);
        end
        function obj = reset(obj)
            obj.alignIndex = obj.numElem + 1;
            obj.buffer(:) = 0;
        end
        function obj = insert(obj, buffer)
            obj.buffer(1:obj.numElem) = obj.buffer(obj.numElem + 1:end);
            obj.buffer(obj.numElem + 1:end) = buffer(:);
        end
        function obj = setAlignIndex(obj, val)
            obj.alignIndex = val;
        end
        function buffer = getAlignBuffer(obj, len)
            if (nargin < 2)
                len = obj.numElem;
            end
            buffer = obj.buffer(obj.alignIndex:obj.alignIndex + len - 1);
        end
        function buffer = getFullBuffer(obj, len)
            if (nargin < 2)
                len = length(obj.buffer);
            end
            buffer = obj.buffer(1:len);
        end
        function out = getAlignIndex(obj)
            out = obj.alignIndex;
        end
        function out = getNumElem(obj)
            out = obj.numElem;
        end
    end
end