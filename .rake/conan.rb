require 'rake'

#
# Tasks for creating a Docker instance for testing
#

class ConanTasks

  include Rake::DSL

  def initialize( opts = {})
    @builds = opts[:builds] || %w( debug release )
    #@build_opts = opts[:build_opts]
    @conan_opts = opts[:opts] || {}
    @conan_scopes = opts[:scopes] || {}
    @conan_settings = opts[:settings] || {}
    @conan_build = opts[:build] || "outdated"
    define_tasks
  end

  def define_tasks

    namespace :conan do

      @builds.each do |build_type|

        namespace build_type.downcase.to_sym do
          build_root = ENV['BUILD_DIR'] || "build"
          build_dir  = "#{build_root}-conan-#{build_type}"

          @conan_settings[:build_type] = build_type.split('_').first
          @conan_opts[:build_gui] = true if build_type =~ /_GUI/

          conan_opts = @conan_opts.each_pair.map { |key,val| "-o %s=%s" % [key,val] } +
                      @conan_settings.each_pair.map { |key,val| "-s %s=%s" % [key,val] } +
                      @conan_scopes.each_pair.map { |key,val| "--scope %s=%s" % [key,val] }

          task :build do
            FileUtils::mkdir build_dir unless FileTest::directory? build_dir
            chdir build_dir do
              sh "conan install %s .. --build=%s" % [conan_opts.join(' '), @conan_build]
              sh "VERBOSE=1 conan build .."
            end

          end


          task :test => :build do
            #sh "cd %s && make unit_test" % build_dir
          end

          task :clean do
            sh "cd %s && make clean" % build_dir
          end
        end

      end

      task :export do
        sh "conan export amarburg/testing"
      end

      task :upload => :export do
        sh "conan upload lsd_slam/master@amarburg/testing"
      end

      namespace :dependencies do
        task :pip_conan do
          sh "pip install conan"
        end

        task :trusty => ["dependencies:trusty", :pip_conan]
        task :xenial => ["dependencies:xenial", :pip_conan]
        task :osx => ["dependencies:osx_brew", :pip_conan]

      end
    end

  end
end
