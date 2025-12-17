import os
from embeding_helpers import get_all_urls, extract_text_from_url, chunk_text, save_chunk_to_qdrant, create_collection

SITEMAP_URL = os.getenv("SITEMAP_URL")


def ingest_book():
    print("Fetching URLs...")
    urls = get_all_urls(SITEMAP_URL)
    
    # --- FILTERING LOGIC ---
    filtered_urls = []
    for u in urls:
        if "tutorial" in u or "blog" in u or "markdown-page" in u or "tags" in u:
            continue
        filtered_urls.append(u)
    
    print(f"\nFiltered URLs count: {len(filtered_urls)} (Original: {len(urls)})")
    for u in filtered_urls:
        print(" -", u)

    create_collection()

    global_id = 1

    for url in filtered_urls:
        print(f"\nProcessing: {url}")
        
        # Try-Except block to prevent crash
        try:
            text = extract_text_from_url(url)
            if not text: continue

            chunks = chunk_text(text)
            print(f" -> Generated {len(chunks)} chunks.")

            for ch in chunks:
                save_chunk_to_qdrant(ch, global_id, url)
                if global_id % 5 == 0: 
                    print(f"   Saved chunk {global_id}...", end="\r")
                global_id += 1
                
        except Exception as e:
            print(f"⚠ Error processing page {url}: {e}")
            continue 

    print(f"\n\n✔ Ingestion completed! Total chunks: {global_id - 1}")

if __name__ == "__main__":
    ingest_book()