#define OPTIMIZED_GFX
#include "Graphics.h"


void Graphics::BeginFrame( Color bg )
{
	// clear the sysbuffer
	std::fill( pSysBuffer, pSysBuffer + Graphics::ScreenHeight * Graphics::ScreenWidth, bg );
}

void Graphics::PutPixel( int x, int y, Color c )
{
	assert( x >= 0 );
	assert( x < int( Graphics::ScreenWidth ) );
	assert( y >= 0 );
	assert( y < int( Graphics::ScreenHeight ) );
	pSysBuffer [Graphics::ScreenWidth * y + x] = c;
}

Color Graphics::GetPixel( int x, int y ) const
{
	assert( x >= 0 );
	assert( x < int( Graphics::ScreenWidth ) );
	assert( y >= 0 );
	assert( y < int( Graphics::ScreenHeight ) );
	return pSysBuffer [Graphics::ScreenWidth * y + x];
}

void Dummy_Template( Graphics& gfx )
{
	gfx.DrawSprite( 0, 0, RectI { 0,0,0,0 }, RectI { 0,0,0,0 }, Surface {}, SpriteEffect::Copy {} );
	gfx.DrawSprite( 0, 0, RectI { 0,0,0,0 }, RectI { 0,0,0,0 }, Surface {}, SpriteEffect::Chroma { Colors::Gray } );
	gfx.DrawSprite( 0, 0, RectI { 0,0,0,0 }, RectI { 0,0,0,0 }, Surface {}, SpriteEffect::Substitution { Colors::Gray ,Colors::Gray } );

}