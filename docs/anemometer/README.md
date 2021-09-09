* Guided by [UK Met office notes](https://www.metoffice.gov.uk/weather/guides/observations/how-we-measure-wind)
* [Automatic weather stations for feed lots](https://www.mla.com.au/globalassets/mla-corporate/research-and-development/program-areas/feeding-finishing-and-nutrition/feedlot-design-manual/043-weather-stations-2016_04_01.pdf)
* [AS2923](https://www.saiglobal.com/PDFTemp/Previews/OSH/As/as2000/2900/2923.pdf)

## Design considerations

Refer to the [anemometer spec sheet](https://cdn.shopify.com/s/files/1/0515/5992/3873/files/6410_SS.pdf)

## Speed computation

V = P(2.25/T) (V = speed in mph, P = no. of pulses per sample period
T = sample period in seconds)

1 mph = 0.868976 knots

V = P(1.955196/T) knots

## Sampling rates

The [UK Met office](https://www.metoffice.gov.uk/weather/guides/observations/how-we-measure-wind) recommends

* sampling wind speed every 0.25 seconds
* The *Gust Speed* is defined by the maximum three second average wind speed occurring in any reporting period
* A better measure of the overall wind intensity is defined by the average speed and direction over the ten minute period leading up to the reporting time. 
* Mean wind over other averaging periods may also be calculated. 
* A gale is defined as a surface wind of mean speed of 34-40 knots, averaged over a period of ten minutes. 
* Terms such as 'severe gale', 'storm', etc are also used to describe winds of 41 knots or greater.

## NMEA output

* [Airmar manual](https://www.airmartechnology.com/uploads/installguide/PB100TechnicalManual_rev1.007.pdf) -- see sentence *$WIVWR* page 19

## Interrupt rate

P = VT/1.955195

Assume gale of V=45 knots, T = 1 second, P = 23 pulses per sec or 1 pulse every 43 mSec.





