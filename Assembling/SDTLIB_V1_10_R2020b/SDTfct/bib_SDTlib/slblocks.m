function blkStruct = slblocks
% This function specifies that the library 'SDTlibV1'
% should appear in the Library Browser with the 
% name 'SDTlib'

    Browser.Library = 'SDTlibV1';
    % 'SDTlibV2' is the name of the library

    Browser.Name = 'SDTlib';
    % 'SDTlib' is the library name that appears
    % in the Library Browser

    blkStruct.Browser = Browser;