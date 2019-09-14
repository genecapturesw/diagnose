# diagnose
GeneCapture Patient Diagnoser  

## Setup
### OSX
`brew cask install anaconda`
TBD

## Spot Detection

### Needs
 - Need list of apriori knowledge (max spot radius, min/max spot distance, cropping info, etc)
 - Landing light requires/spec

### First design
The first output of the spot detection is a sub image of the input FIT file along with a noise value associated with that image. This sub image is produced from the following steps:
 - Image integrity check
   - Did the hardware fail? Is the image abnormal?
 - Crop
   - Use apriori knowledge to exclude information
 - Gaussian blur
 - Find bubbles
   - Output is a binary image where 0 means bubble not present and 1 means bubble present
 - Find landing lights
   - Output is the "spot center" coordinates of landing lights
 - Registration
   - Find height, width, and angle of rotation of the area of interest
   - New spot centers found and radiuses
   - output is a rotated "square" set of spots
 - Noise Background
   - Determine a noise value for the area of interest

We want to output a new FIT file and one noise value.