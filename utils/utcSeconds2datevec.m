function [utcDatevec] = utcSeconds2datevec(utcSeconds)
%UTCSECONDS2DATEVEC Converts seconds since UTC to datevec
utcDatevec = datevec(Constants.DAYS_TO_UTC+utcSeconds/Constants.SECONDS_IN_DAY);
end

