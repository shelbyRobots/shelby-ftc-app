require 'opencv'

include OpenCV

class BeaconDetector

   MIN_COLOR_ZONE_AREA = 0.08 # fraction of total image area
   MIN_BUTTON_AREA = 0.01 # fraction of total image area
   MIN_BUTTON_EDGE_DIST = 20 # pixels from edge of cropped image

   def initialize( image = nil )
      set_image( image ) if image
   end

   def set_image( image )
      # Assert: image is a CvMat object
      # Convert to HSV colorspace to make it easier to
      # threshold certain colors (ig red/blue)
      @source = image
      @image = image.BGR2HSV

      find_colors
      true
   end

   # Is there a blue region in the image
   def blue?
      !@blue_button.nil?
   end

   # CvPoint of the center of the blue button
   def blue_button
      @blue_button
   end

   # Is there a red region in the image
   def red?
      !@red_button.nil?
   end

   # CvPoint of the center of the red button
   def red_button
      @red_button
   end

   private

   def find_largest_object( img, mode )

      barea = 0
      bigr = nil
      h = img.height
      w = img.width
      ima = h * w
      ctr = img.find_contours( :mode => CV_RETR_EXTERNAL, :method => CV_CHAIN_APPROX_NONE )

      # Loop over each blob (contour)
      begin
         cbox = ctr.min_area_rect2
         carea = cbox.size.width * cbox.size.height
         bbox = ctr.bounding_rect

         if mode == :button
            # Only blobs not along edge
            cdn = bbox.x > MIN_BUTTON_EDGE_DIST &&
                     bbox.y > MIN_BUTTON_EDGE_DIST &&
                     bbox.x + bbox.width < w - MIN_BUTTON_EDGE_DIST &&
                     bbox.y + bbox.height < h - MIN_BUTTON_EDGE_DIST &&
                     ctr.contour_area / ima.to_f > MIN_BUTTON_AREA

         else # mode == :light
            # Only blobs that make up a sizable area of the image
            cdn = ctr.contour_area / ima.to_f > MIN_COLOR_ZONE_AREA
         end

         if cdn && ( !bigr || carea > barea )
            bigr = ctr
            barea = carea
         end

         ctr = ctr.h_next
      end while ctr

      return nil unless bigr
      bigr.bounding_rect
   end

   def find_colors
      find_blue
      find_red
   end

   def find_blue
      # Threshold based on color.  White regions match the desired color.  Black do not.
      # We now have a binary image to work with.  Contour detection looks for white blobs
      @blue_areas = @image.in_range( CvScalar.new( 110,100,100 ), CvScalar.new( 130,255,255 ) )

      # There can be several blobs.  Find the largest that fills a certain amount
      # of the image.  These are crude heuristics but should be fine if we control
      # the conditions of when we start searching (ie, appx size of beacon in image
      # frame, etc).
      @blue_light_box = find_largest_object( @blue_areas, :light )
      return unless @blue_light_box

      # Now we want to find the button.  It should be in the blue area
      # we just found so, crop out that area and search for it.
      @blue_crop = @blue_areas.sub_rect( @blue_light_box )

      # The button is a black blob inside a white area.  So we need to invert
      # the image to get a white blob inside a black area.  The problem we'll
      # run into is there will be black regions around the edge (because the beacon
      # is not square) and we don't want those to be considered.
      @blue_button_box = find_largest_object( invert( @blue_crop ), :button )
      return unless @blue_button_box

      @blue_button = CvPoint.new( @blue_button_box.x, @blue_button_box.y )
   end

   def find_red
      # Same game, just a different hue
      @red_areas = @image.in_range( CvScalar.new( 0,100,100 ), CvScalar.new( 10,255,255 ) )
      @red_light_box = find_largest_object( @red_areas, :light )
      return unless @red_light_box

      @red_crop = @red_areas.sub_rect( @red_light_box )
      @red_button_box = find_largest_object( invert( @red_crop ), :button )
      return unless @red_button_box

      @red_button = CvPoint.new( @red_button_box.x, @red_button_box.y )
   end

   def invert( image )
      white = image.clone
      white.set_data( [255] * (image.width * image.height) )
      white - image
   end

   public


   # Saves all the intermediate images for inspection
   def debug
      @blue_areas.save( 'blue-areas.jpg' )
      @red_areas.save( 'red-areas.jpg' )

      if @blue_light_box
         @blue_crop.save( 'blue-crop.jpg' )

         if @blue_button_box

            viz = @source.copy

            viz.rectangle!( CvPoint.new( @blue_light_box.x, @blue_light_box.y ),
                            CvPoint.new(
                                    @blue_light_box.x + @blue_light_box.width,
                                    @blue_light_box.y + @blue_light_box.height ),
                            :color => CvScalar.new( 255, 0, 0 )
                          )

            viz.rectangle!( CvPoint.new( @blue_light_box.x + @blue_button_box.x, @blue_light_box.y + @blue_button_box.y),
                            CvPoint.new(
                                    @blue_light_box.x + @blue_button_box.x + @blue_button_box.width,
                                    @blue_light_box.y + @blue_button_box.y + @blue_button_box.height),
                            :color => CvScalar.new( 255, 0, 255 )
                          )

            viz.save( 'blue-found.png' )

         end
      end

      if @red_light_box
         @red_crop.save( 'red-crop.jpg' )

         if @red_button_box

            viz = @source.copy

            viz.rectangle!( CvPoint.new( @red_light_box.x, @red_light_box.y ),
                            CvPoint.new(
                                    @red_light_box.x + @red_light_box.width,
                                    @red_light_box.y + @red_light_box.height ),
                            :color => CvScalar.new( 0, 0, 255 )
                          )

            viz.rectangle!( CvPoint.new( @red_light_box.x + @red_button_box.x, @red_light_box.y + @red_button_box.y),
                            CvPoint.new(
                                    @red_light_box.x + @red_button_box.x + @red_button_box.width,
                                    @red_light_box.y + @red_button_box.y + @red_button_box.height),
                            :color => CvScalar.new( 255, 0, 255 )
                          )

            viz.save( 'red-found.png' )

         end
      end
   end

   def self.test( file )
      detector = self.new( CvMat.load( file ) )
      detector.debug

      puts file
      if detector.blue?
         puts "Found blue at (" + [ detector.blue_button.x, detector.blue_button.y ].join( ', ' ) + ")"
      else
         puts "No blue here"
      end

      if detector.red?
         puts "Found red at (" + [ detector.red_button.x, detector.red_button.y ].join( ', ' ) + ")"
      else
         puts "No red here"
      end

      puts "\n"
   end

end

BeaconDetector.test( 'beacon-bl-rr.jpg' )
BeaconDetector.test( 'beacon-br-rl.jpg' )
BeaconDetector.test( 'beacon-no-blue.jpg' )
BeaconDetector.test( 'beacon-no-button.jpg' )
BeaconDetector.test( 'beacon-smaller.jpg' )
BeaconDetector.test( 'beacon-bigger.jpg' )

